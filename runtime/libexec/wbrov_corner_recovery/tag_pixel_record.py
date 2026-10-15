"""Canonical image/camera/Tag pixel records. These records are not Core admission."""
from copy import deepcopy
import hashlib
import json
import math

SCHEMA_VERSION=1
PRODUCER='wbrov_apriltag_pixel_record'
CORNER_ORDER='opencv_aruco_canonical_0_1_2_3'

class PixelRecordError(ValueError):
    pass

def require(condition,reason):
    if not condition:raise PixelRecordError(reason)

def _finite(v):
    if type(v) not in (int,float):return False
    try:return math.isfinite(v)
    except OverflowError:return False

def _canonical(obj):
    return json.dumps(obj,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')

def digest(obj):
    return hashlib.sha256(_canonical(obj)).hexdigest()

def _sha(value):
    return type(value) is str and len(value)==64 and all(c in '0123456789abcdef' for c in value)

def _validate_image(image):
    require(type(image) is dict and set(image)=={'stamp_ns','camera_frame','width','height','encoding','sha256'},'invalid image schema')
    require(type(image['stamp_ns']) is int and image['stamp_ns']>0,'invalid image stamp')
    require(type(image['camera_frame']) is str and bool(image['camera_frame']),'invalid camera frame')
    require(type(image['width']) is int and type(image['height']) is int and image['width']>0 and image['height']>0,'invalid image dimensions')
    require(image['encoding'] in ('bgr8','mono8'),'image is not in the detector canonical encoding')
    require(_sha(image['sha256']),'invalid canonical image digest')

def image_record(stamp_ns,camera_frame,width,height,encoding,pixels):
    require(type(pixels) is bytes,'canonical image pixels must be contiguous bytes')
    require(type(width) is int and type(height) is int and width>0 and height>0,'invalid image dimensions')
    require(encoding in ('bgr8','mono8'),'invalid canonical image encoding')
    require(len(pixels)==width*height*(3 if encoding=='bgr8' else 1),'canonical image byte count mismatch')
    out={'stamp_ns':stamp_ns,'camera_frame':camera_frame,'width':width,'height':height,
         'encoding':encoding,'sha256':hashlib.sha256(pixels).hexdigest()}
    _validate_image(out);return out

def camera_record(image,K,D):
    _validate_image(image)
    require(isinstance(K,(list,tuple)) and len(K)==9 and all(_finite(v) for v in K),'invalid camera K')
    require(isinstance(D,(list,tuple)) and len(D) in (0,4,5,8,12,14) and all(_finite(v) for v in D),'invalid camera D')
    require(K[0]>0 and K[4]>0 and abs(K[8]-1)<1e-6 and abs(K[6])<1e-9 and abs(K[7])<1e-9,'invalid pinhole camera matrix')
    out={'model':'opencv_K_D','camera_frame':image['camera_frame'],'width':image['width'],'height':image['height'],
         'K':[float(v) for v in K],'D':[float(v) for v in D]}
    out['sha256']=digest(out);return out

def make_pixel_record(image,tag_id,side_m,corners,K,D):
    require(type(tag_id) is int and tag_id>=0,'invalid Tag identity')
    camera=camera_record(image,K,D)
    out={'schema_version':SCHEMA_VERSION,'producer':PRODUCER,'image':deepcopy(image),'camera':camera,
         'tag_id':tag_id,'side_m':side_m,'corner_order':CORNER_ORDER,'corners_xy':deepcopy(corners),
         'pixel_source':'detector_observation','core_image_verified':False}
    # Normalize finite numeric measurement values once; no pose projection is used.
    require(_finite(side_m) and side_m>0,'invalid Tag side length');out['side_m']=float(side_m)
    require(isinstance(corners,(list,tuple)) and len(corners)==4,'expected four measured corners')
    require(all(isinstance(p,(list,tuple)) and len(p)==2 and all(_finite(v) for v in p) for p in corners),'invalid corner coordinates')
    out['corners_xy']=[[float(v) for v in p] for p in corners]
    out['sha256']=digest(out)
    return validate_pixel_record(out,image['stamp_ns'],tag_id,image['camera_frame'])

def validate_pixel_record(raw,expected_stamp_ns,expected_tag_id,expected_camera_frame):
    require(type(raw) is dict and set(raw)=={'schema_version','producer','image','camera','tag_id','side_m','corner_order',
        'corners_xy','pixel_source','core_image_verified','sha256'},'invalid pixel record schema')
    require(type(raw['schema_version']) is int and raw['schema_version']==SCHEMA_VERSION and raw['producer']==PRODUCER,'invalid pixel record producer/version')
    require(type(raw['tag_id']) is int and raw['tag_id']>=0 and type(expected_tag_id) is int and raw['tag_id']==expected_tag_id,'Tag identity mismatch')
    image=raw['image'];_validate_image(image)
    require(type(expected_stamp_ns) is int and image['stamp_ns']==expected_stamp_ns,'pixel/image stamp mismatch')
    require(image['camera_frame']==expected_camera_frame,'pixel camera frame mismatch')
    camera=raw['camera']
    require(type(camera) is dict and set(camera)=={'model','camera_frame','width','height','K','D','sha256'},'invalid camera record schema')
    rebuilt=camera_record(image,camera['K'],camera['D'])
    require(camera==rebuilt,'camera identity/digest mismatch')
    require(raw['corner_order']==CORNER_ORDER and raw['pixel_source']=='detector_observation','unsupported corner provenance')
    require(raw['core_image_verified'] is False,'detector record cannot claim Core image verification')
    require(_finite(raw['side_m']) and raw['side_m']>0,'invalid Tag side length')
    corners=raw['corners_xy']
    require(type(corners) is list and len(corners)==4,'invalid corner count')
    require(all(type(p) is list and len(p)==2 and all(_finite(v) for v in p) for p in corners),'invalid corner values')
    require(all(0<=p[0]<image['width'] and 0<=p[1]<image['height'] for p in corners),'corner outside source image')
    # Decoder corner order must form a nondegenerate convex quadrilateral.
    cross=[]
    for i in range(4):
        a,b,c=corners[i],corners[(i+1)%4],corners[(i+2)%4]
        cross.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    require(all(v>1e-6 for v in cross),'reversed/nonconvex/degenerate measured corners')
    body={k:v for k,v in raw.items() if k!='sha256'}
    require(_sha(raw['sha256']) and digest(body)==raw['sha256'],'pixel record digest mismatch')
    return deepcopy(raw)

def verify_core_image_record(record,*,stamp_ns,tag_id,camera_frame,width,height,encoding,pixels,K,D):
    """Explicit comparison for an owner that has the actual saved node image.

    Returns a comparison result, not a graph update permission. It does not
    manufacture runtime epoch, GID, database or keyframe provenance.
    """
    valid=validate_pixel_record(record,stamp_ns,tag_id,camera_frame)
    actual=image_record(stamp_ns,camera_frame,width,height,encoding,pixels)
    require(valid['image']==actual,'actual node image does not match detector image')
    require(valid['camera']==camera_record(actual,K,D),'actual node camera does not match detector camera')
    return {'pixel_record_sha256':valid['sha256'],'image_and_camera_match':True,'graph_update_authorized':False}
