"""Publish measured proposals within the same live Core/consumer session."""
from collections import defaultdict,Counter
from copy import deepcopy
from itertools import combinations
import json,hashlib,time
import numpy as np
from measured_bridge import observation,best_old_triplet,propose
from core_pixel_consumer import strict_json,require
from bridge_status import validate_terminal

class LiveBridge:
    def __init__(self,node,consumer,graph,config,out):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.String=String;self.node=node;self.consumer=consumer;self.graph=graph;self.config=config
        self.log=(out/'measured_bridge.jsonl').open('x');self.counts=Counter();self.processed=set();self.rows={};self.references={}
        self.pending=None;self.pending_geometry=None;self.deferred=[]
        self.committed_epochs=set();self.terminals={};self.proposals=0;self.required_revision=0
        qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=node.create_publisher(String,'/wbrov/safe_relocalization/proposal',qos)
        self.sub=node.create_subscription(String,'/wbrov/safe_relocalization/status',self.status,qos)
        self.sub._wbrov_message_info_required=True
    def event(self,kind,**kw):
        self.counts[kind]+=1;self.log.write(json.dumps(dict(event=kind,monotonic_ns=time.monotonic_ns(),**kw),allow_nan=False)+'\n');self.log.flush()
    @staticmethod
    def deferred_score(geometry):
        consistency=geometry['alignment_consistency']
        normalized=(
            consistency['max_translation_m']/.08,
            consistency['max_rotation_deg']/8.,
            consistency['max_z_m']/.02,
            consistency['max_tilt_deg']/2.,
        )
        return (max(normalized),sum(v*v for v in normalized),
                sum(v*v for v in geometry['corner_rms_at_960']))
    def current_score(self,cached):
        from measured_bridge import matrix,distance,tilt
        alignments=[]
        for edge in cached['proposal']['evidence']:
            current,_=self.graph.pose_for_receipt(self.consumer.node_receipts[edge['from_id']])
            old,_=self.graph.pose_for_receipt(self.consumer.node_receipts[edge['to_id']])
            link=matrix(edge)
            alignments.append(matrix(old)@np.linalg.inv(link)@np.linalg.inv(matrix(current)))
        pairwise=[distance(a,b) for a,b in combinations(alignments,2)]
        dz=max(abs(a[2,3]-b[2,3]) for a,b in combinations(alignments,2))
        dt=max(tilt(a,b) for a,b in combinations(alignments,2))
        values=(max(v[0] for v in pairwise)/.08,max(v[1] for v in pairwise)/8.,dz/.02,dt/2.)
        return (max(values),sum(v*v for v in values),cached['score']),dict(
            max_translation_m=max(v[0] for v in pairwise),max_rotation_deg=max(v[1] for v in pairwise),
            max_z_m=dz,max_tilt_deg=dt)
    def publish(self,proposal,geometry,kind,node_id):
        require(self.publisher.get_subscription_count()==1,'Core proposal subscription identity count')
        state=self.graph.current['state'];epoch=proposal['source_reset_epoch']
        require(state['reset_epoch']==epoch and node_id in self.graph.current['poses'],'current graph before proposal')
        self.proposals+=1
        token=hashlib.sha256((self.config['session_token']+str(epoch)+':'+str(node_id)+':'+str(self.proposals)).encode()).hexdigest()
        proposal=deepcopy(proposal)
        proposal['transaction_token']=token
        proposal['observer_graph_revision']=state['graph_revision']
        proposal['created_monotonic_s']=time.monotonic()
        self.pending=deepcopy(proposal);self.pending_geometry=deepcopy(geometry)
        self.event(kind,proposal=proposal,geometry=geometry)
        self.publisher.publish(self.String(data=json.dumps(proposal,allow_nan=False)))
    def proposal_for(self,candidate,tag,epoch,state):
        ids=[i for edge in candidate['evidence'] for i in (edge['from_id'],edge['to_id'])]
        receipts=self.consumer.receipts_for_endpoints(ids,tag)
        corners=self.consumer.corner_observations_for_endpoints(ids,tag)
        return dict(schema_version=3,producer='wbrov_apriltag_safe_relocalizer',session_token=self.config['session_token'],
                    transaction_token='',tag_id=tag,source_reset_epoch=epoch,
                    observer_graph_revision=state['graph_revision'],
                    anchor_database_sha256=self.config['database_sha256'],created_monotonic_s=0.,
                    evidence=candidate['evidence'],accepted_images=receipts,corner_observations=corners)
    def status(self,message,info):
        gid=self.consumer.sources.check('status',incoming=info);s=strict_json(message.data,16384)
        # Preserve authenticated Core bytes before parsing semantic outcomes.
        self.event('received_status',actual_message_gid=gid,raw=message.data,status=s)
        token=s['transaction_token']
        if token in self.terminals:
            require(self.terminals[token]==s,'conflicting repeated terminal result');return
        require(token and self.pending is not None and token==self.pending['transaction_token'],'unowned terminal result')
        validate_terminal(s,self.pending,self.config)
        if s['result']=='committed':
            self.committed_epochs.add(s['core_reset_epoch']);self.required_revision=s['core_graph_revision'];self.deferred=[]
        elif s['reason'].startswith('current reset component needs gravity stabilization ('):
            cached=dict(proposal=deepcopy(self.pending),geometry=deepcopy(self.pending_geometry),
                        score=self.deferred_score(self.pending_geometry))
            key=tuple((e['from_id'],e['to_id']) for e in cached['proposal']['evidence'])
            if key not in {tuple((e['from_id'],e['to_id']) for e in x['proposal']['evidence']) for x in self.deferred}:
                self.deferred.append(cached)
                self.event('deferred_cached',transaction_token=token,source_reset_epoch=s['source_reset_epoch'],
                           observer_graph_revision=s['observer_graph_revision'],score=cached['score'])
        self.terminals[token]=s;self.pending=None;self.pending_geometry=None
        self.event('terminal',actual_message_gid=gid,status=s)
    def allow_ready(self,ns,node_id,epoch):
        if self.graph.last_revision<self.required_revision:return False
        if self.pending is not None:
            require(time.monotonic()-self.pending['created_monotonic_s']<12,'Core proposal did not finish')
            return False
        current_epoch=self.graph.current['state']['reset_epoch']
        # A second tracking reset starts a new graph component. Evidence from
        # the previous component cannot be retried against this one, even if
        # its original endpoints remain in the session receipt table.
        if self.deferred:
            discarded=[x for x in self.deferred if x['proposal']['source_reset_epoch']!=current_epoch]
            if discarded:
                self.deferred=[x for x in self.deferred if x['proposal']['source_reset_epoch']==current_epoch]
                self.event('deferred_discarded_on_reset',epoch=current_epoch,count=len(discarded),
                           source_epochs=sorted({x['proposal']['source_reset_epoch'] for x in discarded}))
        if epoch>current_epoch:return False
        # Core's 200-node floor prevents a clearly immature merge, while this
        # replay-derived 325-node retry window lets the post-reset gravity graph
        # settle before it is tied to the previous component. Re-submit the best
        # earlier three-view measurement once that window is complete, even if
        # the Tag has left the current image. All six original
        # accepted-image receipts and corner hashes remain bound to the same Core
        # session and are revalidated atomically by Core before any graph change.
        if self.deferred and epoch==current_epoch and epoch not in self.committed_epochs:
            state=self.graph.current['state']
            if (state['reset_epoch']==epoch and len(self.graph.current['poses'])>=325 and
                    state['graph_revision']>max(x['proposal']['observer_graph_revision'] for x in self.deferred)):
                ranked=[]
                for cached in self.deferred:
                    score,consistency=self.current_score(cached)
                    ranked.append((score,cached,consistency))
                ranked.sort(key=lambda x:x[0]);score,cached,consistency=ranked[0]
                self.event('deferred_selection',node_id=node_id,epoch=epoch,score=score,
                           current_alignment_consistency=consistency,candidates=len(ranked))
                self.deferred=[item[1] for item in ranked[1:]]
                self.publish(cached['proposal'],cached['geometry'],'deferred_retry',node_id)
                return False
        if ns in self.processed:return True
        self.processed.add(ns)
        bindings=[r for (i,t),r in self.consumer.bindings.items() if i==node_id]
        for binding in bindings:
            pose,revision=self.graph.pose_for_receipt(binding['accepted_image_receipt'])
            row=observation(binding,pose);self.rows[(epoch,node_id,binding['tag_id'])]=row
        if epoch<current_epoch:
            if bindings:self.event('late_historical_observation',node_id=node_id,epoch=epoch,
                                   current_epoch=current_epoch,tags=sorted(b['tag_id'] for b in bindings))
            return True
        if epoch==0 or epoch in self.committed_epochs:return True
        for binding in bindings:
            tag=binding['tag_id'];key=(epoch,tag)
            if key not in self.references:
                # A component that resets again before its first safe commit
                # is not part of the trusted old map. Prefer the most recent
                # committed epoch with this Tag, then fall back to epoch zero.
                for old_epoch in sorted(({0}|self.committed_epochs),reverse=True):
                    if old_epoch>=epoch:continue
                    older=[r for (e,i,t),r in self.rows.items() if e==old_epoch and t==tag]
                    if len(older)<3:continue
                    # Refresh historical poses from that epoch's final Core snapshot.
                    for r in older:
                        pose,_=self.graph.pose_for_receipt(r['binding']['accepted_image_receipt'])
                        from measured_bridge import matrix
                        r['body']=matrix(pose)
                    reference=best_old_triplet(older)
                    if reference is not None:
                        self.references[key]=reference
                        self.event('historical_reference',epoch=epoch,tag_id=tag,
                                   source_epoch=old_epoch,observations=len(older))
                        break
            if key not in self.references:continue
            current=[r for (e,i,t),r in self.rows.items() if e==epoch and t==tag]
            for r in current:
                pose,_=self.graph.pose_for_receipt(r['binding']['accepted_image_receipt'])
                from measured_bridge import matrix
                r['body']=matrix(pose)
            try:candidate,reason=propose(self.references[key],current)
            except ValueError as exc:candidate=None;reason=str(exc)
            self.event('geometry',node_id=node_id,stamp_ns=str(ns),epoch=epoch,tag_id=tag,reason=reason,candidate=candidate)
            if candidate is None:continue
            state=self.graph.current['state'];require(state['reset_epoch']==epoch and node_id in self.graph.current['poses'],'current graph before proposal')
            self.publish(self.proposal_for(candidate,tag,epoch,state),candidate,'proposal',node_id);return False
        return True
    def close(self):
        try:
            self.event('closed',proposals=self.proposals,committed_epochs=sorted(self.committed_epochs),
                       counts=dict(self.counts),pending_transaction=self.pending,deferred_candidate=self.deferred,
                       complete=self.pending is None)
        finally:self.log.close()
        return self.pending is None
