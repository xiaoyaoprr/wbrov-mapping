"""Compare a detector measurement with Core's accepted-image receipt.

This value comparison neither authenticates ROS transport nor proves that a
node still exists in the current graph. Those checks belong to the Core-facing
consumer. Original detector records remain unchanged; no graph edge is emitted.
"""
from copy import deepcopy
import hashlib,json,math,re
from tag_pixel_record import validate_pixel_record,digest

class ReceiptJoinError(ValueError):pass
def require(ok,reason):
    if not ok:raise ReceiptJoinError(reason)
def finite(v):
    if type(v) not in (int,float):return False
    try:return math.isfinite(v)
    except OverflowError:return False
def sha(v):return type(v) is str and re.fullmatch('[0-9a-f]{64}',v) is not None
def integer_string(v,maximum,positive=False):
    require(type(v) is str and re.fullmatch('0|[1-9][0-9]*',v) is not None,'noncanonical decimal integer')
    require(len(v)<=20,'integer overflow');n=int(v)
    require(int(positive)<=n<=maximum,'integer out of bounds');return n

def parse_receipt(raw):
    def pairs(items):
        result={}
        for key,value in items:
            require(key not in result,'duplicate JSON key');result[key]=value
        return result
    def bad_constant(_):raise ReceiptJoinError('nonfinite JSON value')
    require(type(raw) is str and len(raw)<=16384,'invalid receipt text size')
    return json.loads(raw,object_pairs_hook=pairs,parse_constant=bad_constant)

def compare_accepted_image(receipt,pixel_record,*,expected_session_sha256,expected_database_sha256,expected_body_from_camera):
    require(sha(expected_session_sha256) and expected_session_sha256!='0'*64 and sha(expected_database_sha256),'invalid expected session/database')
    fields={'schema_version','producer','node_id','map_id','header_stamp_ns','reset_epoch','image_width','image_height','encoding',
            'rgb_sha256','camera_frame','K','D','body_from_camera','rgb_publisher_gid','camera_info_publisher_gid',
            'session_token_sha256','database_sha256','source_scope','pixel_role','saved_jpeg_bytes_are_original_pixels'}
    require(type(receipt) is dict and set(receipt)==fields,'unexpected receipt schema')
    require(type(receipt['schema_version']) is int and receipt['schema_version']==1 and receipt['producer']=='wbrov_core_accepted_image_input','wrong receipt producer/version')
    require(type(receipt['node_id']) is int and 0<receipt['node_id']<=2147483647,'invalid node ID')
    require(type(receipt['map_id']) is int and 0<=receipt['map_id']<=2147483647,'invalid map ID')
    ns=integer_string(receipt['header_stamp_ns'],2**63-1,True);epoch=integer_string(receipt['reset_epoch'],2**64-1)
    require(receipt['session_token_sha256']==expected_session_sha256 and receipt['database_sha256']==expected_database_sha256,'wrong session/database')
    require(receipt['source_scope']=='managed_ros_rgbd' and receipt['pixel_role']=='authenticated_pre_memory_input','unsupported image origin')
    require(type(receipt['saved_jpeg_bytes_are_original_pixels']) is int and receipt['saved_jpeg_bytes_are_original_pixels']==0,'invalid JPEG claim')
    require(type(receipt['camera_frame']) is str and 0<len(receipt['camera_frame'])<=256,'invalid camera frame')
    for key in ('rgb_publisher_gid','camera_info_publisher_gid'):
        value=receipt[key]
        require(type(value) is str and re.fullmatch('[0-9a-f]{32,64}',value) is not None and len(value)%2==0 and set(value)!={'0'},'invalid source GID')
    require(len(receipt['rgb_publisher_gid'])==len(receipt['camera_info_publisher_gid']),'inconsistent GID storage')
    require(sha(receipt['rgb_sha256']),'invalid RGB hash')
    require(all(type(receipt[k]) is int and receipt[k]>0 for k in ('image_width','image_height')),'invalid image dimensions')
    require(receipt['encoding'] in ('bgr8','mono8'),'invalid encoding')
    for key,size in (('K',9),('body_from_camera',12)):
        require(type(receipt[key]) is list and len(receipt[key])==size and all(finite(v) for v in receipt[key]),'invalid numeric vector '+key)
    require(type(receipt['D']) is list and len(receipt['D']) in (0,4,5,8,12,14) and all(finite(v) and v==0 for v in receipt['D']),'unsupported distortion')
    require(type(expected_body_from_camera) in (list,tuple) and len(expected_body_from_camera)==12 and all(finite(v) for v in expected_body_from_camera),'invalid expected transform')
    require(receipt['body_from_camera']==list(expected_body_from_camera),'camera extrinsic mismatch')
    require(type(pixel_record) is dict and type(pixel_record.get('tag_id')) is int,'invalid detector record')
    pixel=validate_pixel_record(pixel_record,ns,pixel_record['tag_id'],receipt['camera_frame'])
    image=pixel['image'];camera=pixel['camera']
    require(image['width']==receipt['image_width'] and image['height']==receipt['image_height'] and image['encoding']==receipt['encoding'] and image['sha256']==receipt['rgb_sha256'],'Core image identity mismatch')
    require(camera['K']==receipt['K'] and camera['D']==receipt['D'],'Core camera identity mismatch')
    return dict(schema_version=1,producer='wbrov_core_pixel_identity_comparison',accepted_image_identity_match=True,
        image_stamp_ns=ns,keyframe_id=receipt['node_id'],map_id=receipt['map_id'],reset_epoch=epoch,
        tag_id=pixel['tag_id'],pixel_observation=deepcopy(pixel),core_receipt_sha256=digest(receipt),
        session_token_sha256=expected_session_sha256,database_sha256=expected_database_sha256,
        body_from_camera=deepcopy(receipt['body_from_camera']),receipt_transport_verified=False,
        requires_current_core_graph_validation=True,graph_update_authorized=False,usable_as_absolute_anchor=False)
