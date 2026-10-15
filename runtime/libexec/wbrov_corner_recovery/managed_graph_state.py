"""Consume the existing Core schema-3 graph state with actual source metadata."""
from copy import deepcopy
import math
import time
from core_pixel_consumer import strict_json, require

class ManagedGraphState:
    def __init__(self,node,consumer,config,record):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.consumer=consumer;self.config=config;self.record=record;self.by_epoch={};self.current=None
        self.last_revision=-1;self.last_epoch=-1;self.received=0;self.rejected=0
        self.subscription=node.create_subscription(String,config['sources']['state']['topic'],self.receive,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription._wbrov_message_info_required=True

    def receive(self,message,info):
        try:
            gid=self.consumer.sources.check('state',incoming=info)
            state=strict_json(message.data,8*1024*1024)
            require(set(state)=={'schema_version','producer','reset_epoch','epoch_start_stamp_ns','graph_revision','database_sha256',
                'database_binding_valid','session_token_sha256','graph_snapshot_valid','graph_poses'},'invalid graph state schema')
            require(type(state['schema_version']) is int and state['schema_version']==3 and
                state['producer']=='wbrov_rtabmap_safe_relocalization_core','invalid graph state producer')
            require(state['database_binding_valid'] is True and state['graph_snapshot_valid'] is True,'graph state not valid')
            require(state['session_token_sha256']==self.config['session_token_sha256'] and
                state['database_sha256']==self.config['database_sha256'],'graph state session/database mismatch')
            for key in ('reset_epoch','epoch_start_stamp_ns','graph_revision'):
                require(type(state[key]) is int and 0<=state[key]<=2**64-1,'invalid graph integer')
            require(state['reset_epoch']>=self.last_epoch and state['graph_revision']>=self.last_revision,'graph state went backwards')
            if state['graph_revision']==self.last_revision:
                require(self.current is not None and self.current['state']==state,'conflicting graph revision')
                return
            rows=state['graph_poses'];require(type(rows) is list and len(rows)<=20000,'invalid graph pose count')
            poses={}
            for row in rows:
                require(type(row) is dict and set(row)=={'id','translation','quaternion_xyzw'},'invalid graph pose schema')
                require(type(row['id']) is int and 0<row['id']<=2147483647 and row['id'] not in poses,'invalid/duplicate graph node')
                for key,count in (('translation',3),('quaternion_xyzw',4)):
                    require(type(row[key]) is list and len(row[key])==count and
                        all(type(v) in (int,float) and math.isfinite(v) for v in row[key]),'invalid graph pose values')
                require(abs(sum(v*v for v in row['quaternion_xyzw'])-1)<.002,'nonunit graph quaternion')
                poses[row['id']]=row
            self.last_epoch=state['reset_epoch'];self.last_revision=state['graph_revision'];self.received+=1
            # Both indexes reference the one validated, privately decoded JSON
            # object. No need to duplicate every pose in the current snapshot.
            self.current=dict(state=state,poses=poses,publisher_gid=gid,arrival_monotonic_ns=time.monotonic_ns())
            self.by_epoch[state['reset_epoch']]=self.current
            self.record(state,message.data,gid)
        except (ValueError,TypeError,KeyError,OverflowError,RecursionError) as exc:
            self.rejected+=1
            self.consumer.rejected(exc)

    def compact(self,receipts):
        if self.current is None:return
        retained={}
        for receipt in receipts:retained.setdefault(int(receipt['reset_epoch']),set()).add(receipt['node_id'])
        current_epoch=self.current['state']['reset_epoch']
        for epoch,snapshot in list(self.by_epoch.items()):
            # Different DDS topics can arrive out of order. The first receipt
            # of a later epoch is the single-writer barrier proving all earlier
            # admitted receipts have already been delivered to this consumer.
            if epoch==current_epoch or epoch>=self.consumer.last_receipt_epoch:continue
            ids=retained.get(epoch,set())
            if not ids:del self.by_epoch[epoch];continue
            # This is a derived retained-pose cache, never a new authenticated
            # graph state. Full current snapshots and raw evidence are intact.
            self.by_epoch[epoch]=dict(snapshot,state={k:v for k,v in snapshot['state'].items() if k!='graph_poses'},
                poses={i:p for i,p in snapshot['poses'].items() if i in ids},compacted=True)

    def pose_for_receipt(self,receipt):
        epoch=int(receipt['reset_epoch']);snapshot=self.by_epoch.get(epoch)
        require(snapshot is not None and receipt['node_id'] in snapshot['poses'],'receipt node absent from retained epoch graph')
        self.consumer.sources.check('state',claimed=snapshot['publisher_gid'])
        return deepcopy(snapshot['poses'][receipt['node_id']]),snapshot['state']['graph_revision']
