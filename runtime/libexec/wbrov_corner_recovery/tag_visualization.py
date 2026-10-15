"""Read-only display publisher inside the existing authenticated Tag consumer."""
from copy import deepcopy
import json,time
from core_pixel_consumer import ManagedSources,strict_json,require
from tag_visualization_geometry import TagPlacementStore,registry_index,SCHEMA

class TagVisualizationPublisher:
    def __init__(self,node,consumer,config,observer,emit):
        import cv2
        import numpy as np
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        # Family comes from the real detector object, not an unverified incoming
        # string. The old immutable pixel schema itself has no family field.
        expected=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        require(np.array_equal(observer.refiner.dictionary.bytesList,expected.bytesList),'unsupported actual Tag detector dictionary')
        self.String=String;self.consumer=consumer;self.config=config;self.emit=emit
        self.store=TagPlacementStore(config);self.registry=None;self.last_registry_ns=0
        self.last_revision=-1;self.last_payload=None;self.invalidated=None;self.closed=False
        self.max_age_ns=3_000_000_000
        state_spec=dict(config['sources']['state'],topic='/wbrov/submaps/registry')
        self.sources=ManagedSources(node,{'registry':state_spec})
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=node.create_publisher(String,'/wbrov/tag_visualization/state',qos)
        self.subscription=node.create_subscription(String,'/wbrov/submaps/registry',self.receive_registry,qos)
        self.subscription._wbrov_message_info_required=True
        self.timer=node.create_timer(.25,self.tick)

    def publish(self,payload):
        encoded=json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False)
        if encoded==self.last_payload:return
        self.publisher.publish(self.String(data=encoded));self.last_payload=encoded
        self.emit(dict(event='tag_visualization',registry_revision=payload['registry_revision'],
            visualization_valid=payload['visualization_valid'],tags=len(payload['tags']),reason=payload.get('reason'),
            graph_update_authorized=False))

    def clear(self,reason):
        self.publish(dict(schema=SCHEMA,session_short_hash=self.config['session_token_sha256'][:16],
            session_token_sha256=self.config['session_token_sha256'],registry_revision=max(0,self.last_revision),
            visualization_valid=False,reason=str(reason),tags=[]))

    def consume_event(self,event):
        if self.closed:return
        if event.get('event')=='sources_invalidated' or (event.get('event')=='rejected' and 'conflicting' in event.get('reason','')):
            self.invalidated='Tag evidence source/identity conflict';self.clear(self.invalidated);return
        if event.get('event')!='bound':return
        try:
            key=(event['keyframe_id'],event['tag_id'])
            require(key in self.consumer.bindings,'display binding absent from actual consumer')
            binding=self.consumer.bindings[key]
            self.consumer.check_receipt_context(binding['accepted_image_receipt'])
            current=self.consumer.sources.all_current()
            require(binding['transport_proof']['current_endpoints']==current,'display binding sources changed')
            self.store.add(binding)
        except Exception as exc:
            self.invalidated='Tag display evidence invalid: '+str(exc);self.clear(self.invalidated)

    def receive_registry(self,message,info):
        if self.closed:return
        try:
            self.sources.check('registry',incoming=info)
            registry=strict_json(message.data,8*1024*1024)
            revision,_=registry_index(registry,self.config['session_token_sha256'])
            require(revision>=self.last_revision,'display registry revision went backwards')
            if revision==self.last_revision:
                require(self.registry==registry,'conflicting display registry revision')
            self.registry=deepcopy(registry);self.last_revision=revision;self.last_registry_ns=time.monotonic_ns()
            self.store.dirty=True
            self.tick()
        except (ValueError,TypeError,KeyError,OverflowError,RecursionError) as exc:
            self.registry=None
            self.clear('registry rejected: '+str(exc))

    def tick(self):
        if self.closed:return
        try:
            require(self.invalidated is None,self.invalidated or 'invalid Tag display')
            require(self.registry is not None,'waiting for authenticated submap registry')
            require(time.monotonic_ns()-self.last_registry_ns<=self.max_age_ns,'submap registry stale')
            self.sources.all_current();self.consumer.sources.all_current()
            if self.store.dirty:self.publish(self.store.snapshot(self.registry))
        except (ValueError,TypeError,KeyError,OverflowError,RecursionError) as exc:
            self.clear(str(exc))

    def close(self):
        if self.closed:return
        self.clear('Tag consumer closed');self.closed=True;self.timer.cancel()
