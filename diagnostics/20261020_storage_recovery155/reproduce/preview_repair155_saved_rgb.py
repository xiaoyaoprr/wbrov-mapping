"""Test-only recorded RGB publisher. No camera, TF, controller or map writes."""
from pathlib import Path
import hashlib,json,sys,time
r=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery/submap_tag_20261015/round7_deferred_20261018')
pkg=r/'mapping_round6_candidate155_recovery'
sys.path.insert(0,str(pkg/'tools'))
from deferred_tag_observations import map_pixels
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
base=r/'repair153_replay/field'
receipt=json.loads((base/'receipt-00000374.json').read_text())
manifest=base/'input-00000374.json'
assert sha(manifest)==receipt['input_manifest_sha256']
d=json.loads(manifest.read_text());source=Path(d['directory'])/d['data_file']
assert sha(source)==d['sha256'][d['data_file']]
images=list(map_pixels(source,d['frames']))
assert images and images[-1][0]==int(receipt['last_source_ns'])
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init()
node=Node('wbrov_saved_rgb_acceptance_only')
pub=node.create_publisher(Image,'/wbrov/test/recorded_rgb',
    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
index=0
def publish():
    global index
    stamp,(width,height,_,pixels)=images[index%len(images)]
    message=Image();message.header.frame_id='recorded_camera_optical'
    message.header.stamp.sec=stamp//10**9;message.header.stamp.nanosec=stamp%10**9
    message.height=height;message.width=width;message.encoding='bgr8'
    message.step=width*3;message.data=pixels
    pub.publish(message);index+=1
    if index==1 or index==10:print(json.dumps({'published':index,'historical_only':True,
        'source_ns':stamp,'width':width,'height':height,'pixels_sha256':hashlib.sha256(pixels).hexdigest()}),flush=True)
node.create_timer(.2,publish)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
