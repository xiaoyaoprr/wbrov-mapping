"""Publish measured proposals within the same live Core/consumer session."""
from collections import defaultdict,Counter
from copy import deepcopy
from itertools import combinations
import json,hashlib,time,os
import numpy as np
from measured_bridge import observation,best_old_triplet,propose
from core_pixel_consumer import strict_json,require
from bridge_status import validate_terminal,validate_observing
from deferred_retry_window import DeferredRetryWindow
from reference_policy import historical_options,choose_candidate

class LiveBridge:
    def __init__(self,node,consumer,graph,config,out):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.String=String;self.node=node;self.consumer=consumer;self.graph=graph;self.config=config
        self.log=(out/'measured_bridge.jsonl').open('x');self.counts=Counter();self.processed=set();self.rows={};self.references={}
        self.pending=None;self.pending_geometry=None;self.deferred=[]
        self.history_epoch=None;self.history_dirty=True;self.historic_keys=set()
        self.completed_for_retention=set();self.retention_passes=0;self.retention_reclaimed=0
        self.early_retry=self.early_retry_enabled(os.environ)
        self.retry_window=DeferredRetryWindow()
        self.committed_epochs=set();self.terminals={};self.proposals=0;self.required_revision=0
        qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=node.create_publisher(String,'/wbrov/safe_relocalization/proposal',qos)
        self.sub=node.create_subscription(String,'/wbrov/safe_relocalization/status',self.status,qos)
        self.sub._wbrov_message_info_required=True
    @staticmethod
    def early_retry_enabled(environment):
        flag=environment.get('WBROV_PRIVATE_EARLY_TAG_RETRY','0')
        require(flag in ('0','1'),'invalid private early retry flag')
        # The retired GRAVITY_CERTIFICATE flag never controls Core acceptance.
        # Core always validates its certificate and all subsequent geometry.
        return flag=='1'
    def event(self,kind,**kw):
        self.counts[kind]+=1;self.log.write(json.dumps(dict(event=kind,monotonic_ns=time.monotonic_ns(),**kw),allow_nan=False)+'\n');self.log.flush()
    @staticmethod
    def deferred_score(geometry):
        from measured_bridge import normalized_alignment
        consistency=geometry['alignment_consistency']
        normalized=normalized_alignment(consistency)
        return (max(normalized),sum(v*v for v in normalized),
                sum(v*v for v in geometry['corner_rms_at_960']))
    def current_score(self,cached):
        from measured_bridge import matrix,body_support_points,alignment_metrics,normalized_alignment
        alignments=[];current_poses=[]
        for edge in cached['proposal']['evidence']:
            current,_=self.graph.pose_for_receipt(self.consumer.node_receipts[edge['from_id']])
            old,_=self.graph.pose_for_receipt(self.consumer.node_receipts[edge['to_id']])
            link=matrix(edge)
            current_poses.append(matrix(current))
            alignments.append(matrix(old)@np.linalg.inv(link)@np.linalg.inv(current_poses[-1]))
        metric=alignment_metrics(alignments,body_support_points(current_poses))
        values=normalized_alignment(metric)
        return (max(values),sum(v*v for v in values),cached['score']),metric
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
        if s.get('result')=='observing':
            # Prepared geometry is not a terminal result or a committed map.
            # Keep the same pending proposal and its original timeout; exact
            # repeated ACKs cannot authorize another proposal or extend life.
            validate_observing(s,self.pending,self.config)
            self.event('observing',actual_message_gid=gid,status=s)
            return
        validate_terminal(s,self.pending,self.config)
        if s['result']=='committed':
            self.committed_epochs.add(s['core_reset_epoch']);self.required_revision=s['core_graph_revision'];self.deferred=[]
        elif s['reason'].startswith(('current reset component needs gravity stabilization (',
                                     'temporary recovery segment has not passed source-time/measured-gravity stability (')):
            cached=dict(proposal=deepcopy(self.pending),geometry=deepcopy(self.pending_geometry),
                        score=self.deferred_score(self.pending_geometry))
            key=tuple((e['from_id'],e['to_id']) for e in cached['proposal']['evidence'])
            if key not in {tuple((e['from_id'],e['to_id']) for e in x['proposal']['evidence']) for x in self.deferred}:
                self.deferred.append(cached)
                if self.early_retry and len(self.deferred)>8:
                    self.deferred.sort(key=lambda x:(x['geometry']['reference_preference'],x['score']));self.deferred=self.deferred[:8]
                    self.event('deferred_capacity_bounded',capacity=8)
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
        if self.early_retry and epoch==current_epoch:
            # allow_ready is called only after an exact Core receipt and its
            # authenticated graph pose exist. No wall-time or pose extrapolation.
            receipt=self.consumer.node_receipts[node_id]
            require(int(receipt['reset_epoch'])==epoch and int(receipt['header_stamp_ns'])==ns,
                    'retry clock must match exact accepted receipt')
            self.retry_window.observe(epoch,node_id,ns)
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
        # Default production behavior is unchanged. The private source-clock
        # retry window only schedules a new proposal; Core must independently
        # pass its complete early certificate and all geometry/shadow guards.
        # Core validates a measured early certificate below 200 nodes. The
        # default replay-derived 325-node retry window lets the post-reset gravity graph
        # settle before it is tied to the previous component. Re-submit the best
        # earlier three-view measurement once that window is complete, even if
        # the Tag has left the current image. All six original
        # accepted-image receipts and corner hashes remain bound to the same Core
        # session and are revalidated atomically by Core before any graph change.
        if self.deferred and epoch==current_epoch and epoch not in self.committed_epochs:
            state=self.graph.current['state']
            rejected_revision=max(x['proposal']['observer_graph_revision'] for x in self.deferred)
            retry_due=(self.retry_window.eligible(epoch,state['graph_revision'],rejected_revision) if self.early_retry else
                len(self.graph.current['poses'])>=325 and state['graph_revision']>rejected_revision)
            if state['reset_epoch']==epoch and retry_due:
                if self.early_retry:self.retry_window.checked()
                ranked=[]
                for cached in self.deferred:
                    score,consistency=self.current_score(cached)
                    if self.early_retry and score[0]>1.:continue
                    ranked.append((score,cached,consistency))
                if ranked:
                    ranked.sort(key=lambda x:(x[1]['geometry']['reference_preference'],x[0]));score,cached,consistency=ranked[0]
                    self.event('deferred_selection',node_id=node_id,epoch=epoch,score=score,
                               current_alignment_consistency=consistency,candidates=len(ranked),
                               private_early_retry=self.early_retry,source_span_ns=self.retry_window.latest-self.retry_window.first if self.early_retry else None,
                               attempt=self.retry_window.attempts+1 if self.early_retry else None)
                    self.deferred=[item for item in self.deferred if item is not cached]
                    if self.early_retry:self.retry_window.submitted()
                    self.publish(cached['proposal'],cached['geometry'],'deferred_retry',node_id)
                    return False
        if ns in self.processed:return True
        self.processed.add(ns)
        bindings=[r for (i,t),r in self.consumer.bindings.items() if i==node_id]
        for binding in bindings:
            pose,revision=self.graph.pose_for_receipt(binding['accepted_image_receipt'])
            row=observation(binding,pose);self.rows[(epoch,node_id,binding['tag_id'])]=row
        if epoch<current_epoch:
            if bindings:self.history_dirty=True
            if bindings:self.event('late_historical_observation',node_id=node_id,epoch=epoch,
                                   current_epoch=current_epoch,tags=sorted(b['tag_id'] for b in bindings))
            return True
        if epoch==0 or epoch in self.committed_epochs:return True
        for binding in bindings:
            tag=binding['tag_id']
            options=[dict(epoch=k[2],reference=ref) for k,ref in self.references.items() if k[:2]==(epoch,tag)]
            if not options:
                options=historical_options(self,tag,epoch)
                for rank,option in enumerate(options):
                    old_epoch=option['epoch'];self.references[(epoch,tag,old_epoch)]=option['reference']
                    self.event('historical_reference',epoch=epoch,tag_id=tag,source_epoch=old_epoch,
                        observations=option['observations'],preference_rank=rank,policy='stable_origin_then_recent')
            if not options:continue
            current=[r for (e,i,t),r in self.rows.items() if e==epoch and t==tag]
            for r in current:
                pose,_=self.graph.pose_for_receipt(r['binding']['accepted_image_receipt'])
                from measured_bridge import matrix
                r['body']=matrix(pose)
            def record_attempt(option,rank,candidate,reason):
                self.event('geometry',node_id=node_id,stamp_ns=str(ns),epoch=epoch,tag_id=tag,
                    source_epoch=option['epoch'],preference_rank=rank,reason=reason,candidate=candidate)
            candidate=choose_candidate(options,current,record_attempt)
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
