"""Authenticated Tag-corner detector, receipt consumer and graph observer.

The recording harness uses the default node names and frame-ready topic.  A
production launcher may supply different names and set ``frame_ready_topic``
to null; proposal admission and all source/hash checks stay identical.
"""
from pathlib import Path
from collections import OrderedDict
import hashlib,json,os,signal,sys
import cv2,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String,UInt64MultiArray
from cv_bridge import CvBridge
from verified_pixel_observer import VerifiedPixelObserver
from core_pixel_consumer import make_consumer,ManagedSources
from managed_graph_state import ManagedGraphState
from message_info_executor import MessageInfoExecutor
from live_bridge import LiveBridge
from startup_sources import wait_for_sources
from history_retention import compact
from tag_visualization import TagVisualizationPublisher

root=Path(__file__).resolve().parent;config=json.loads(Path(sys.argv[1]).read_text());out=Path(sys.argv[2]);out.mkdir()
fixture_mode=bool(config.get('fixture_mode',True))
if fixture_mode:
    assert os.environ.get('ROS_DOMAIN_ID')=='197' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
cv2.setNumThreads(1);stop=False
def stopping(*_):
    global stop
    stop=True
signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
nodes=config.get('nodes',{})
detector=Node(nodes.get('detector','private_pixel_detector'))
owner=Node(nodes.get('consumer','authenticated_core_pixel_consumer'))
qos=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
pixel_pub=detector.create_publisher(String,config['sources']['pixels']['topic'],qos)
ready_topic=config.get('frame_ready_topic','/private/recorded_frame_ready')
ready_pub=(owner.create_publisher(UInt64MultiArray,ready_topic,qos)
           if isinstance(ready_topic,str) and ready_topic else None)
events=(out/'events.jsonl').open('x');payloads=(out/'detector.jsonl').open('x');states=(out/'graph_states.jsonl').open('x')
pixel_consumed=set();pending_ready={};ready_count=0;detected_count=0;last_retention_key=None
tag_visualizer=None
def emit(event):
    events.write(json.dumps(event,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n');events.flush()
    if event['event'] in ('bound','no_measured_tags') and int(event['image_stamp_ns']) in pending_ready:
        pixel_consumed.add(int(event['image_stamp_ns']))
    if tag_visualizer is not None:tag_visualizer.consume_event(event)
with (out/'startup_sources.jsonl').open('x') as admission_log:
    def record_admission(event):
        admission_log.write(json.dumps(event,sort_keys=True)+'\n');admission_log.flush()
    admitted_sources=wait_for_sources(owner,config['sources'],lambda:stop,record_admission)
consumer=make_consumer(owner,config,emit)
# Source checks remain strict after admission, including every callback GID.
# A later replacement cannot be treated as another startup or cleared here.
consumer.sources.bound.update(admitted_sources)
def record_state(state,raw,gid):
    name=out/('graph_epoch_'+str(state['reset_epoch'])+'.json');temp=name.with_suffix('.tmp');temp.write_text(raw);temp.replace(name)
    states.write(json.dumps(dict(reset_epoch=state['reset_epoch'],graph_revision=state['graph_revision'],
        pose_count=len(state['graph_poses']),maximum_id=max((r['id'] for r in state['graph_poses']),default=0),
        sha256=hashlib.sha256(raw.encode()).hexdigest(),actual_message_gid=gid))+'\n');states.flush()
graph=ManagedGraphState(owner,consumer,config,record_state)
proposer=Node(nodes.get('proposer','apriltag_safe_relocalizer'));live=LiveBridge(proposer,consumer,graph,config,out)
sources=ManagedSources(detector,{k:config['sources'][k] for k in ('rgb','camera_info')})
images=OrderedDict();infos=OrderedDict();bridge=CvBridge();observer=VerifiedPixelObserver()
tag_visualizer=TagVisualizationPublisher(owner,consumer,config,observer,emit)
image_buffer_capacity=int(config.get('image_buffer_capacity',128))
assert 16 <= image_buffer_capacity <= 256
buffer_dropped={'rgb':0,'camera_info':0}
def stamp(message):return int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec)
def process(ns):
    global detected_count
    # Detect only images that Core has actually admitted. The live high-rate
    # RGB stream can contain many frames dropped by RTAB-Map's synchronizer;
    # processing all of them would waste CPU and fill the pixel wait queue.
    if ns not in images or ns not in infos or ns not in consumer.receipts:return
    image=images.pop(ns);info=infos.pop(ns)
    assert image.encoding=='bgr8' and image.header.frame_id==info.header.frame_id==config['camera_frame']
    assert image.width==info.width and image.height==info.height
    sources.all_current()
    pixels=bridge.imgmsg_to_cv2(image,desired_encoding='passthrough').copy()
    payload=observer.observe(pixels,stamp_ns=ns,camera_frame=info.header.frame_id,
        K=[float(v) for v in info.k],D=[float(v) for v in info.d],
        side_m=float(config.get('tag_side_m',.08)),min_side_px=float(config.get('min_tag_side_px',8)))
    payloads.write(json.dumps(payload,sort_keys=True,allow_nan=False)+'\n');payloads.flush()
    pending_ready[ns]=[r['tag_id'] for r in payload['observations']]
    pixel_pub.publish(String(data=json.dumps(payload,allow_nan=False)));detected_count+=1
def buffer(role,store,ns,message):
    assert ns not in store
    store[ns]=message
    while len(store)>image_buffer_capacity:
        store.popitem(last=False);buffer_dropped[role]+=1
def rgb(message,info):
    sources.check('rgb',incoming=info);ns=stamp(message);buffer('rgb',images,ns,message);process(ns)
def camera(message,info):
    sources.check('camera_info',incoming=info);ns=stamp(message);buffer('camera_info',infos,ns,message);process(ns)
subs=[detector.create_subscription(Image,config['sources']['rgb']['topic'],rgb,qos),
      detector.create_subscription(CameraInfo,config['sources']['camera_info']['topic'],camera,qos)]
for subscription in subs:subscription._wbrov_message_info_required=True
def ready():
    global ready_count,last_retention_key
    # A Core receipt normally follows the corresponding RGB callbacks, so the
    # timer completes the three-way join without modifying the Core consumer.
    for ns in list(images):
        if ns in infos and ns in consumer.receipts:process(ns)
    for ns,tags in list(pending_ready.items()):
        if ns not in consumer.receipts or ns not in pixel_consumed:continue
        receipt=consumer.receipts[ns]['receipt'];epoch=int(receipt['reset_epoch']);node_id=receipt['node_id']
        if epoch not in graph.by_epoch or node_id not in graph.by_epoch[epoch]['poses']:continue
        if not all((node_id,tag) in consumer.bindings for tag in tags):continue
        if not live.allow_ready(ns,node_id,epoch):continue
        pose,revision=graph.pose_for_receipt(receipt)
        if ready_pub is not None:
            ready_pub.publish(UInt64MultiArray(data=[ns,node_id,epoch]))
        pending_ready.pop(ns);pixel_consumed.remove(ns);ready_count+=1
        live.completed_for_retention.add(ns)
        if not tags:
            receipt=consumer.receipts.pop(ns,None)
            if receipt is not None:consumer.node_receipts.pop(receipt['receipt']['node_id'],None)
        emit(dict(event='frame_ready',image_stamp_ns=str(ns),node_id=node_id,reset_epoch=epoch,graph_revision=revision,
            tag_ids=tags,graph_update_authorized=False))
    key=(ready_count,graph.last_epoch,consumer.last_receipt_epoch)
    if key!=last_retention_key:
        compact(live,pending_ready);last_retention_key=key
timer=owner.create_timer(.01,ready)
executor=MessageInfoExecutor();executor.add_node(detector);executor.add_node(owner);executor.add_node(proposer)
parts=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()
(out/'ready.json').write_text(json.dumps({'pid':os.getpid(),'start_ticks':parts[19]}))
try:
    while rclpy.ok() and not stop:executor.spin_once(timeout_sec=.05)
    assert stop,'unexpected ROS shutdown'
finally:
    # Complete every cleanup attempt, preserving any primary replay exception.
    primary=sys.exc_info()[1];cleanup_errors=[];bridge_complete=False
    try:bridge_complete=live.close()
    except BaseException as exc:cleanup_errors.append(repr(exc))
    operations=[tag_visualizer.close,consumer.close,lambda:executor.remove_node(owner),lambda:executor.remove_node(detector),
        lambda:executor.remove_node(proposer),proposer.destroy_node,owner.destroy_node,detector.destroy_node,
        executor.shutdown,lambda:rclpy.shutdown() if rclpy.ok() else None,events.close,payloads.close,states.close]
    for operation in operations:
        try:operation()
        except BaseException as exc:cleanup_errors.append(repr(exc))
    with (out/'cleanup.json').open('x') as f:json.dump(dict(primary_exception=repr(primary) if primary else None,
        cleanup_errors=cleanup_errors,bridge_complete=bridge_complete),f,indent=2)
    if primary is None:
        assert not cleanup_errors and bridge_complete,('incomplete consumer shutdown',cleanup_errors,bridge_complete)
with (out/'RESULT.json').open('x') as f:json.dump(dict(bridge_proposals=live.proposals,bridge_terminals=live.terminals,committed_epochs=sorted(live.committed_epochs),state='closed',detected_images=detected_count,ready_frames=ready_count,
    counts=consumer.counts,graph_states=graph.received,graph_state_rejections=graph.rejected,source_compromised=consumer.sources.compromised,
    pending_ready=len(pending_ready),graph_epochs=sorted(graph.by_epoch),actual_sensor_dds_callbacks=True,
    actual_verified_pixel_observer=True,actual_graph_state_source_checked=True,
    fixture_mode=fixture_mode,image_buffer_capacity=image_buffer_capacity,
    buffer_dropped=buffer_dropped,retention=dict(passes=live.retention_passes,reclaimed=live.retention_reclaimed,
        rows=len(live.rows),processed=len(live.processed),bindings=len(consumer.bindings),receipts=len(consumer.receipts),
        completed_tombstones=len(live.completed_for_retention),references=len(live.references),
        historical_poses=sum(len(s['poses']) for e,s in graph.by_epoch.items() if s is not graph.current)),hardware_started=False),f,indent=2)
