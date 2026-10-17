"""Recorded native ROS VO + trusted main/independent submaps + optional real Tag.

Private localhost domain only. No camera, SDK, control, production DB or launcher.
384x240 recorded input, NOT production 960x600/GPU-camera/RViz performance proof.
The source models a bounded single pending 1Hz mapping frame awaiting exact VO ACK.
"""
from pathlib import Path
import argparse,array,hashlib,json,math,os,re,secrets,signal,struct,subprocess,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,Imu,PointCloud2
from replay_audit import audit,graph_components
from replay_timing import timing_evidence
from rtabmap_msgs.msg import RGBDImage,OdomInfo,MapGraph
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String,UInt64
from tf2_ros import StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage

assert os.environ.get('ROS_DOMAIN_ID')=='207' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
parser=argparse.ArgumentParser();parser.add_argument('--order',choices=['ab','ba'],default='ab')
parser.add_argument('--count',type=int,default=1200);parser.add_argument('--start',type=int,default=1000)
parser.add_argument('--cycles',type=int,default=1)
parser.add_argument('--latest-only',action='store_true')
parser.add_argument('--odom-profile',choices=['historical-latest','deployed-inline'],default='historical-latest',
    help='deployed-inline matches the current dual-stream launcher VO parameters; recorded dimensions remain 384x240.')
parser.add_argument('--blank-cut-s',type=float,default=0.,help='Blank each side of repeated-video cuts; actual native loss/reset, never supplied poses.')
parser.add_argument('--runtime-receipt',type=Path,required=True,help='Exact task-owned candidate receipt, pinned for the whole run.')
parser.add_argument('--tag-enabled',action='store_true',help='Separate optional Tag case; default no Tag process.')
parser.add_argument('--no-controlled-faults',action='store_true')
parser.add_argument('--short-blank-s',type=float,default=.25)
parser.add_argument('--long-blank-s',type=float,default=1.2)
args=parser.parse_args();assert 200<=args.count<=2400 and 0<=args.start and args.start+args.count<=2421
assert 1<=args.cycles<=3 and 0<=args.blank_cut_s<=5
assert 0<args.short_blank_s<=.5 and .5<=args.long_blank_s<=5
args.latest_only=True
os.environ['WBROV_DEPTH_RECOVERY']='1'
assert args.cycles==1 or args.latest_only,'Endurance is a single candidate run, not a repeated A/B benchmark'
ROOT=Path('/home/exocean/.cache/wbrov_repairs/20260923_tag_recovery')
LIVE=Path('/home/exocean/.cache/kcap_navigation_candidates/20260920/KCAP_Navigation_20260920_candidate_v12')
SOURCE=Path('/home/exocean/.cache/wbrov_mapping_builds/offline_corner_20260907/good')
sys.path.insert(0,str(LIVE))
from navigation.completed_map_jobs import file_hash,write_atomic,robot_activity
sys.path.insert(0,str(ROOT))
from analyze_closed_logs import describe
os.umask(0o077)
TASK=ROOT/'submap_tag_20261015'
TASK.mkdir(mode=0o700,exist_ok=True)
receipt_path=args.runtime_receipt.resolve()
assert receipt_path.resolve().is_relative_to(TASK) and receipt_path.name=='runtime_receipt.json'
receipt=json.loads(receipt_path.read_text());runtime=Path(receipt['runtime'])
assert runtime.resolve().is_relative_to(TASK) and runtime.name=='experimental_runtime'
assert receipt.get('single_main_map_policy') is True and receipt.get('inertial_translation') is False
assert receipt.get('temporary_submaps') is True and receipt.get('apriltag_visualization') is True
for name,pin in receipt['files'].items():assert file_hash(runtime/name)==pin,name
assert not robot_activity(),robot_activity()
camera=json.loads((SOURCE/'camera.json').read_text());assert (camera['width'],camera['height'])==(384,240)
stride=8+384*240*7;frames=[]
with (SOURCE/'frames.bin').open('rb') as stream:
    for index in range(args.start,args.start+args.count):
        stream.seek(48+index*stride);frames.append((struct.unpack('<Q',stream.read(8))[0],index))
assert all(b[0]>a[0] for a,b in zip(frames,frames[1:]))
first,last=frames[0][0],frames[-1][0]
imu_rows=[]
for line in (SOURCE/'imu.txt').read_text().splitlines():
    vals=line.split();ns=int(vals[0])
    if first-500_000_000<=ns<=last+500_000_000:imu_rows.append((ns,list(map(float,vals[1:]))))
assert imu_rows and all(len(row)==37 for _,row in imu_rows)
source_identity={p.name:(p.stat().st_size,p.stat().st_mtime_ns) for p in SOURCE.iterdir() if p.name in ('frames.bin','imu.txt','camera.json')}
out=TASK/('recorded_submap_tag_'+str(time.monotonic_ns()));out.mkdir(mode=0o700)
# Seal the actually executed driver before any long-lived child is created.
(out/'replay_driver.py').write_bytes(Path(__file__).read_bytes())
(out/'replay_audit.py').write_bytes((Path(__file__).parent/'replay_audit.py').read_bytes())
(out/'submap_replay_audit.py').write_bytes((Path(__file__).parent/'submap_replay_audit.py').read_bytes())
(out/'replay_timing.py').write_bytes((Path(__file__).parent/'replay_timing.py').read_bytes())
report=dict(schema='wbrov.recorded-submap-tag-replay.v1',phase='starting',root=str(out),runtime=str(runtime),
    runtime_receipt_path=str(receipt_path),runtime_receipt_sha256=file_hash(receipt_path),driver_sha256=file_hash(Path(__file__)),
    timing_audit_sha256=file_hash(Path(__file__).parent/'replay_timing.py'),
    no_tag_case=not args.tag_enabled,depth_geometry_recovery=True,inertial_translation=False,independent_local_submaps=True,
    early_gravity_measured_certificate=True,
    order=args.order,odom_profile=args.odom_profile,source_start=args.start,source_count=args.count,cycles=args.cycles,blank_cut_s=args.blank_cut_s,cases={},hardware_started=False,
    production_changed=False,field_validated=False,
    scope='Real native ROS nodes and message_filters; original 384x240 recorded RGBD/IMU. VO recomputed. Optional Tag and graph-display workload only; no UI validation, GPU camera, RViz renderer, high-resolution mapping, or physical ground truth.')
write_atomic(out/'result.json',report);print(json.dumps(report),flush=True)

def ns_of(msg):return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec

def gravity(acc):
    norm=np.linalg.norm(acc)
    if not math.isfinite(norm) or not 8<=norm<=12:return None
    B=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float);a=B@np.asarray(acc)/norm
    roll=math.atan2(a[1],a[2]);pitch=math.atan2(-a[0],math.hypot(a[1],a[2]))
    cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    desired=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
    return Rotation.from_matrix(np.array([[0,-1,0],[1,0,0],[0,0,1]])@desired@B).as_quat().tolist()

def command(exe,name,params,remaps):
    cmd=[str(exe),'--ros-args','-r','__node:='+name]
    for a,b in remaps.items():cmd+=['-r',a+':='+b]
    for key,value in params.items():
        text=str(value).lower() if isinstance(value,bool) else str(value)
        if '/' in key or key in ('safe_relocalization_session_token','safe_relocalization_database_sha256') or 'to_base' in key:text="'"+text+"'"
        cmd+=['-p',key+':='+text]
    return cmd

def run_case(label,enabled):
    assert not robot_activity(),robot_activity()
    folder=out/label;folder.mkdir(mode=0o700)
    rclpy.init();node=rclpy.create_node('zed_rgbd_dual_stream_publisher')
    stopping=[False]
    def request_stop(*_):stopping[0]=True
    for signum in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(signum,request_stop)
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE)
    latched=QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rgbd_pub=node.create_publisher(RGBDImage,'/zed/rgbd_image',qos)
    imu_pub=node.create_publisher(Imu,'/zed/imu/data',qos)
    topics=['/zed/mapping/rgb/image_rect_color','/zed/mapping/depth/depth_registered','/zed/mapping/rgb/camera_info']
    high_qos=QoSProfile(depth=4,reliability=ReliabilityPolicy.RELIABLE)
    map_pubs=[node.create_publisher(typ,topic,high_qos) for typ,topic in zip((Image,Image,CameraInfo),topics)]
    odoms={};infos={};accepted=[];graphs=[];markers=[];states=[];continuity=[];clouds=[];phases={};published_frames=[]
    registries=[];submap_graphs=[];submap_clouds=[];submap_markers=[];tag_states=[];main_tag_markers=[];submap_tf_edges=[];dynamic_subscriptions={}
    event_log=(folder/'events.jsonl').open('x',buffering=1)
    def event(kind,row):event_log.write(json.dumps(dict(kind=kind,**row),separators=(',',':'))+'\n')
    def on_odom(msg):
        assert ns_of(msg) not in odoms,'Repeated output source stamp'
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        odoms[ns_of(msg)]=dict(ns=ns_of(msg),arrival_ns=time.time_ns(),xyz=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w],cov=list(msg.pose.covariance))
    def on_info(msg):
        ns=ns_of(msg)
        row=dict(ns=ns,arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),lost=msg.lost,
            inliers=msg.inliers,features=msg.features,matches=msg.matches,
            inliers_distribution=getattr(msg,'inliers_distribution',None),covariance=list(msg.covariance),
            icp_correspondences=msg.icp_correspondences,
            icp_inliers_ratio=msg.icp_inliers_ratio,icp_structural_complexity=msg.icp_structural_complexity,
            icp_structural_distribution=msg.icp_structural_distribution,estimation_s=msg.time_estimation,
            camera_dimensions=[[m.camera_info.width,m.camera_info.height] for group in msg.local_bundle_models for m in group.models],
            fault_phase=phases.get(ns,'not_a_published_source_stamp'))
        infos[ns]=row;event('odom_info',row)
    def on_accepted(msg):
        row=dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),receipt=json.loads(msg.data))
        accepted.append(row);event('accepted_input',row)
    def graph_row(msg):
        ids=list(msg.poses_id);links=[[int(link.from_id),int(link.to_id)] for link in msg.links]
        pose_values=[[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w] for p in msg.poses]
        return dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),ns=ns_of(msg),frame_id=msg.header.frame_id,nodes=len(ids),
            node_ids=ids,links=links,components=graph_components(ids,links),
            poses_sha256=hashlib.sha256(json.dumps(pose_values,separators=(',',':')).encode()).hexdigest())
    def on_graph(msg):
        row=graph_row(msg)
        graphs.append(row);event('public_map_graph',row)
    def on_continuity(msg):
        row=dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),status=json.loads(msg.data))
        continuity.append(row);event('continuity_status',row)
    def cloud_row(msg):
        return dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),ns=ns_of(msg),frame_id=msg.header.frame_id,
            width=msg.width,height=msg.height,point_step=msg.point_step,row_step=msg.row_step,data_bytes=len(msg.data),
            data_sha256=hashlib.sha256(msg.data).hexdigest())
    def on_cloud(msg):
        row=cloud_row(msg)
        clouds.append(row);event('public_cloud_map',row)
    def on_submap_graph(msg,key):
        row=dict(graph_row(msg),key=key);submap_graphs.append(row);event('local_submap_graph',row)
    def on_submap_cloud(msg,key):
        row=dict(cloud_row(msg),key=key);submap_clouds.append(row);event('local_submap_cloud',row)
    def marker_row(msg):
        return dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),markers=[dict(
            frame_id=m.header.frame_id,ns=ns_of(m),namespace=m.ns,id=m.id,action=m.action,type=m.type,
            position=[m.pose.position.x,m.pose.position.y,m.pose.position.z],text=m.text,
            points=[[p.x,p.y,p.z] for p in m.points]) for m in msg.markers])
    def on_submap_markers(msg,key):
        row=dict(marker_row(msg),key=key);submap_markers.append(row);event('local_submap_markers',row)
    def on_main_tag_markers(msg):
        row=marker_row(msg);main_tag_markers.append(row);event('main_tag_markers',row)
    def on_tag_state(msg):
        row=dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),state=json.loads(msg.data))
        tag_states.append(row);event('tag_visualization_state',row)
    def on_registry(msg):
        raw=json.loads(msg.data);row=dict(arrival_ns=time.time_ns(),arrival_steady_ns=time.monotonic_ns(),registry=raw)
        registries.append(row);event('submap_registry',row)
        assert raw.get('schema')=='wbrov.submaps.v1' and len(raw.get('submaps',[]))<=4096,'Unexpected registry schema/capacity'
        for group in raw['submaps']:
            key=group['key'];assert re.fullmatch(r'm\d+_e\d+_c[1-9]\d*',key),'Invalid dynamic topic identity'
            for suffix,typ,callback in [('graph',MapGraph,on_submap_graph),('cloud',PointCloud2,on_submap_cloud),('display',MarkerArray,on_submap_markers)]:
                topic=f'/wbrov/submaps/{key}/{suffix}';assert group[suffix+'_topic']==topic,'Unexpected submap topic'
                if topic not in dynamic_subscriptions:
                    dynamic_subscriptions[topic]=node.create_subscription(typ,topic,lambda message,k=key,fn=callback:fn(message,k),latched)
    def on_tf(msg):
        for transform in msg.transforms:
            if transform.header.frame_id.startswith('wbrov_submap_') or transform.child_frame_id.startswith('wbrov_submap_'):
                row=dict(arrival_ns=time.time_ns(),parent=transform.header.frame_id,child=transform.child_frame_id,ns=ns_of(transform))
                submap_tf_edges.append(row);event('unexpected_submap_tf',row)
    def on_state(msg):
        raw=json.loads(msg.data);states.append(dict(arrival_ns=time.time_ns(),reset_epoch=raw.get('reset_epoch'),graph_revision=raw.get('graph_revision')))
    node.create_subscription(Odometry,'/odom',on_odom,qos)
    # Full OdomInfo is intentionally observed to authenticate actual VO dimensions
    # and measured visual/geometric quality. Scheduler lateness is measured and
    # the result is not represented as camera performance if this adds latency.
    node.create_subscription(OdomInfo,'/odom_info',on_info,qos)
    node.create_subscription(String,'/wbrov/mapping_continuity/status',on_continuity,latched)
    node.create_subscription(PointCloud2,'/cloud_map',on_cloud,latched)
    node.create_subscription(String,'/wbrov/accepted_image_input',on_accepted,latched)
    node.create_subscription(MapGraph,'/mapGraph',on_graph,latched)
    node.create_subscription(String,'/wbrov/submaps/registry',on_registry,latched)
    node.create_subscription(String,'/wbrov/tag_visualization/state',on_tag_state,latched)
    node.create_subscription(MarkerArray,'/wbrov/tags/main_markers',on_main_tag_markers,latched)
    node.create_subscription(TFMessage,'/tf',on_tf,qos)
    node.create_subscription(TFMessage,'/tf_static',on_tf,latched)
    node.create_subscription(MarkerArray,'/mapping/graph_display',lambda msg:markers.append(dict(arrival_ns=time.time_ns(),count=len(msg.markers))),reliable)
    if args.cycles==1:node.create_subscription(String,'/wbrov/safe_relocalization/state',on_state,latched)
    else:
        # The real Tag consumer still reads/authenticates the complete graph.
        # The frame source only needs the scalar epoch; don't make its timer
        # decode an ever-growing second copy of the complete graph JSON.
        node.create_subscription(UInt64,'/wbrov/reset_epoch',lambda msg:states.append(dict(arrival_ns=time.time_ns(),reset_epoch=msg.data)),latched)
    tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id=camera['frame']
    tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,camera['base_from_camera'][:3])
    tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=camera['base_from_camera'][3:]
    broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(tf)
    children={};child_identities={};samples=[];timing=[];published=[];published_set=set();planned=[];map_sent=[];dropped=[];pending=None
    faults=[];progress_clock=0.;cycle_index=0;started_steady=time.monotonic()
    datahash=hashlib.sha256();last_health=0;bindings={};result=dict(enabled=enabled,source_metric_info_topic='/odom_info',slam_info_topic='/odom_info',
        map_ack_policy='finite unit pose on exact /odom stamp only; 750ms monotonic deadline; reliable depth4, modeled after pinned publisher')
    env=dict(os.environ,LD_LIBRARY_PATH=str(runtime/'lib')+':/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu',
             PYTHONPATH=str(runtime/'local/lib/python3.10/dist-packages')+':'+str(runtime/'lib/python3.10/site-packages')+':'+os.environ.get('PYTHONPATH',''))
    def start(role,cmd):
        trace=folder/('trace_'+role);trace.mkdir(mode=0o700)
        with (folder/(role+'.log')).open('x') as log:
            p=subprocess.Popen(cmd,cwd=folder,env=dict(env,WBROV_PRIVATE_OPTIMIZER_TRACE=str(trace)),
                stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children[role]=p
        child_identities[role]=dict(pid=p.pid,start_ticks=Path('/proc',str(p.pid),'stat').read_text().rsplit(')',1)[1].split()[19])
        write_atomic(folder/'owned_children.json',dict(owner_pid=os.getpid(),owner_start_ticks=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19],children=child_identities))
    def health(force=False):
        nonlocal last_health,progress_clock
        assert not stopping[0],'Owned replay stop requested'
        for role,p in children.items():assert p.poll() is None,role+' exited '+str(p.returncode)
        if not force and time.monotonic()-last_health<2:return
        last_health=time.monotonic();assert not robot_activity(tuple(p.pid for p in children.values())),'Robot activity appeared'
        resource=dict(steady_ns=time.monotonic_ns(),wall_ns=time.time_ns(),roles={})
        for role,p in children.items():
            fields=Path('/proc',str(p.pid),'stat').read_text().rsplit(')',1)[1].split()
            resource['roles'][role]=dict(cpu_ticks=int(fields[11])+int(fields[12]),rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE'))
            maps=Path('/proc',str(p.pid),'maps').read_text().splitlines()
            bound={row.split(None,5)[-1] for row in maps if '/librtabmap_' in row}
            for path in bound:
                assert Path(path).parent==runtime/'lib','Mixed native runtime: '+path
                if path not in bindings.get(role,[]):
                    assert file_hash(Path(path))==receipt['files'][str(Path(path).relative_to(runtime))]
            if bound:bindings[role]=sorted(bound)
        samples.append(resource)
        if args.cycles>1:
            memory={line.split(':')[0]:int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemAvailable:','SwapFree:'))}
            fs=os.statvfs(folder);free_bytes=fs.f_bavail*fs.f_frsize
            assert memory['MemAvailable']>2*1024**3,'Stop owned replay before memory pressure affects user work'
            assert free_bytes>25*1024**3,'Stop owned replay before disk space falls below guard'
            assert sum(row['rss_bytes'] for row in resource['roles'].values())<7*1024**3,'Candidate total RSS exceeded 7GiB safety budget'
            if time.monotonic()-progress_clock>=15:
                progress_clock=time.monotonic()
                progress=dict(phase='running',owner_pid=os.getpid(),cycle_index=cycle_index,cycles=args.cycles,wall_elapsed_s=time.monotonic()-started_steady,
                    published=len(published),unique_odom=len(odoms),lost_infos=sum(row['lost'] for row in infos.values()),map_published=len(map_sent),map_accepted=len(accepted),
                    epochs=sorted({int(row['reset_epoch']) for row in states}),resource=resource,mem_available_bytes=memory['MemAvailable'],disk_free_bytes=free_bytes,
                    last_odom_ns=max(odoms,default=0),last_accepted_arrival_ns=accepted[-1]['arrival_ns'] if accepted else None)
                write_atomic(folder/'progress.json',progress)
                print(json.dumps({key:value for key,value in progress.items() if key not in ('resource',)}),flush=True)
    def valid(ns):
        if ns not in odoms or ns not in infos:return False
        row=odoms[ns];info=infos[ns]
        return not info['lost'] and (info['inliers']>=15 or info['icp_correspondences']>=80) and np.isfinite(row['xyz']+row['q']+row['cov']).all() and abs(np.linalg.norm(row['q'])-1)<1e-3 and max(row['cov'][::7])<9999
    def flush_map():
        nonlocal pending
        if pending is None:return
        ns,rgb,depth,ci,queued_steady=pending
        age=(time.time_ns()-ns)*1e-9
        row=odoms.get(ns)
        ack_valid=row is not None and np.isfinite(row['xyz']+row['q']).all() and abs(float(np.dot(row['q'],row['q']))-1)<=1e-3
        if time.monotonic_ns()-queued_steady>750_000_000:dropped.append(dict(ns=ns,reason='ACK_deadline_750ms',age_s=age));pending=None
        elif ack_valid:
            for pub,msg in zip(map_pubs,(rgb,depth,ci)):pub.publish(msg)
            map_sent.append(dict(ns=ns,age_s=age,publish_ns=time.time_ns(),rgb_sha256=hashlib.sha256(rgb.data).hexdigest()))
            pending=None
    odom_params=dict(frame_id='base_link',odom_frame_id='odom',publish_tf=True,qos=2,qos_camera_info=2,qos_imu=2,
        subscribe_rgbd=True,approx_sync=False,wait_imu_to_init=True,always_check_imu_tf=False,managed_external_reset_disabled=True,
        managed_expected_imu_node='zed_rgbd_dual_stream_publisher',managed_expected_imu_namespace='/',managed_expected_imu_frame=camera['frame'],
        managed_expected_imu_to_base='0 0 0 -0.5 0.5 -0.5 0.5',always_process_most_recent_frame=True,
        bounded_latest_frame_handoff=enabled,publish_null_when_lost=False,imu_queue_size=200)
    odom_params.update({'Odom/ResetCountdown':8,'Odom/GuessMotion':'false','Odom/FillInfoData':'true','Odom/ImageDecimation':1,'OdomF2M/BundleAdjustment':1,'OdomF2M/BundleAdjustmentMaxFrames':3,
        'OdomF2M/MaxSize':2000,'Optimizer/Iterations':5,'Optimizer/GravitySigma':.3,'GFTT/MinDistance':3,'Vis/GridRows':2,'Vis/GridCols':3,
        'Vis/MaxDepth':1.0,'Vis/MinInliers':15,'Vis/MinInliersDistribution':.01,'Vis/DepthAsMask':'true','Vis/CorGuessMatchToProjection':'true'})
    if args.odom_profile=='deployed-inline':
        odom_params.update(always_process_most_recent_frame=False,bounded_latest_frame_handoff=False,topic_queue_size=1,sync_queue_size=10)
        odom_params.update({'Vis/MaxDepth':1.5,'Vis/MinInliersDistribution':0.0,'Vis/CorGuessWinSize':40})
    token=secrets.token_hex(32)
    slam_params=dict(database_path=str(folder/'private_mapping.db'),frame_id='base_link',map_frame_id='map',subscribe_depth=True,
        subscribe_rgb=True,subscribe_scan=False,subscribe_odom_info=True,odom_sensor_sync=False,approx_sync=False,topic_queue_size=10,
        sync_queue_size=10,qos_image=1,qos_camera_info=1,qos_odom=2,wait_for_transform=.5,latch=True,reset_quarantine_enabled=True,
        unannounced_jump_quarantine_enabled=True,reset_quarantine_lost_count=8,reset_quarantine_odom_info_max_age_sec=.5,
        reset_quarantine_max_unannounced_translation=.5,reset_quarantine_max_unannounced_rotation_deg=30.,
        reset_quarantine_expected_odom_info_node='rtabmap_rgbd_odometry',reset_quarantine_expected_odom_namespace='/',
        reset_quarantine_expected_sensor_node='zed_rgbd_dual_stream_publisher',reset_quarantine_expected_sensor_namespace='/',
        reset_quarantine_expected_sensor_frame=camera['frame'],reset_quarantine_expected_sensor_to_base='0 0 0 -0.5 0.5 -0.5 0.5',
        reset_quarantine_expected_imu_node='zed_rgbd_dual_stream_publisher',reset_quarantine_expected_imu_namespace='/',
        reset_quarantine_expected_imu_frame=camera['frame'],safe_relocalization_enabled=True,safe_relocalization_expected_proposal_node='safe_reloc_gid_frontend',
        safe_relocalization_expected_proposal_namespace='/',safe_relocalization_session_token=token,safe_relocalization_database_sha256='0'*64,
        safe_relocalization_start_new_map_on_first_update=False,safe_relocalization_timeout_sec=30.,
        mapping_continuity_vo_width=384,mapping_continuity_vo_height=240)
    slam_params.update({'Rtabmap/DetectionRate':5,'Rtabmap/StartNewMapOnLoopClosure':'false','Rtabmap/CrossMapLoopPending':'true',
        'RGBD/LinearUpdate':.05,'RGBD/AngularUpdate':.05,'RGBD/NeighborLinkRefining':'false','RGBD/ProximityBySpace':'false',
        'RGBD/OptimizeMaxError':1.,'RGBD/OptimizeMaxErrorRepairRadius':0,'RGBD/CreateOccupancyGrid':'true',
        'Optimizer/LandmarksIgnored':'true','Optimizer/PriorsIgnored':'true','Rtabmap/LoopGPS':'false','Reg/Force3DoF':'false',
        'Optimizer/Iterations':20,'Optimizer/Strategy':1,'Optimizer/VarianceIgnored':'false','Mem/IncrementalMemory':'true',
        'Mem/InitWMWithAllNodes':'true','Mem/ReduceGraph':'false','Mem/RehearsalIdUpdatedToNewOne':'false','Mem/RehearsalSimilarity':1.,
        'Rtabmap/TimeThr':0,'Rtabmap/MemoryThr':0,'Rtabmap/CreateIntermediateNodes':'false','Mem/UseOdomGravity':'false',
        'Optimizer/GravitySigma':.3,'Mem/SaveDepth16Format':'true','Vis/MaxDepth':1.5,'Vis/MinInliers':25,
        'Grid/RangeMin':.2,'Grid/RangeMax':.8,'Grid/CellSize':.03,'Grid/DepthDecimation':2,'Grid/NoiseFilteringRadius':.05,'Grid/NoiseFilteringMinNeighbors':3})
    remaps=dict(zip(['rgb/image','depth/image','rgb/camera_info'],topics))
    remaps.update(dict(zip(['reset_quarantine_rgb','reset_quarantine_depth','reset_quarantine_camera_info'],topics)))
    remaps.update({'odom':'/odom','reset_quarantine_odom':'/odom','odom_info':'/odom_info','reset_quarantine_odom_info':'/odom_info','imu':'/zed/imu/data'})
    # Preserve exact effective parameters; this replay is not camera/UI acceptance.
    write_atomic(folder/'node_parameters.json',dict(odom_profile=args.odom_profile,odom=odom_params,
        slam={k:v for k,v in slam_params.items() if k!='safe_relocalization_session_token'}))
    try:
        start('odom',command(runtime/'bin/rgbd_odometry','rtabmap_rgbd_odometry',odom_params,{'rgbd_image':'/zed/rgbd_image','imu':'/zed/imu/data'}))
        start('slam',command(runtime/'bin/rtabmap','rtabmap',slam_params,remaps))
        if args.tag_enabled:
            tagroot=runtime/'libexec/wbrov_corner_recovery'
            config=json.loads((tagroot/'production_config_template.json').read_text())
            config.update(session_token=token,session_token_sha256=hashlib.sha256(token.encode()).hexdigest(),database_sha256='0'*64)
            write_atomic(folder/'tag_config.json',config)
            start('tag',['/usr/bin/python3','-B',str(tagroot/'consumer.py'),str(folder/'tag_config.json'),str(folder/'tag')])
        start('display',['/usr/bin/python3','-B',str(runtime/'libexec/rtabmap_graph_display.py'),'--ros-args','-r','__node:=rtabmap_graph_display'])
        views=folder/'submap_views';views.mkdir(mode=0o700)
        start('submap_display',['/usr/bin/python3','-B',str(runtime/'libexec/rtabmap_submap_display.py'),'--rviz-output-dir',str(views)])
        deadline=time.monotonic()+35
        while True:
            health()
            if rgbd_pub.get_subscription_count()>=1 and imu_pub.get_subscription_count()>=2 and all(p.get_subscription_count()>=2 for p in map_pubs) and (not args.tag_enabled or (folder/'tag/ready.json').exists()):break
            assert time.monotonic()<deadline,'Full-stack input subscriptions not ready'
            rclpy.spin_once(node,timeout_sec=.02)
        assert not [name for name,_ in node.get_service_names_and_types() if name.endswith(('/reset_odom','/reset_odom_to_pose'))]
        # Give authenticated participants a discovery window before first frame.
        until=time.monotonic()+2
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01);health()
        schedule=sorted([(ns,0,row) for ns,row in imu_rows]+[(ns,1,index) for ns,index in frames],key=lambda r:(r[0],r[1]))
        origin=schedule[0][0];anchor_wall=time.time_ns()+1_000_000_000;anchor_steady=time.monotonic_ns()+1_000_000_000
        offset=anchor_wall-origin;next_map=first;result['stamp_offset_ns']=offset
        # Every cycle has a known constant source-time translation. Adjacent
        # cycles are separated by a normal IMU sample period; never retimestamp
        # based on how far processing fell behind the fixed monotonic schedule.
        imu_step=int(np.median(np.diff([row[0] for row in imu_rows])))
        cycle_period=schedule[-1][0]-origin+imu_step
        assert cycle_period>last-first and cycle_period*args.cycles<14_400_000_000_000
        source_duration=(last-first)*1e-9
        assert source_duration>8, 'Recording slice too short for separated fault/recovery phases'
        short_start=source_duration*.2;long_start=source_duration*.5
        assert short_start+args.short_blank_s<long_start and long_start+args.long_blank_s<source_duration
        result.update(cycle_period_ns=cycle_period,cycles=args.cycles,blank_cut_s=args.blank_cut_s,
            controlled_fault_plan=[] if args.no_controlled_faults else [
                dict(kind='blank_rgb_depth_preserved',begin_s=short_start,duration_s=args.short_blank_s),
                dict(kind='blank_rgb_and_depth',begin_s=long_start,duration_s=args.long_blank_s)],
            tag_enabled=args.tag_enabled,depth_geometry_recovery=True,inertial_translation=False)
        def events():
            for cycle in range(args.cycles):
                for original,kind,payload in schedule:yield cycle,original,original+cycle*cycle_period,kind,payload
        with (SOURCE/'frames.bin').open('rb') as stream:
            for cycle_index,original,logical,kind,payload in events():
                target=anchor_steady+logical-origin
                while time.monotonic_ns()<target:
                    rclpy.spin_once(node,timeout_sec=min(.005,max(0,(target-time.monotonic_ns())*1e-9)));flush_map();health()
                ns=logical+offset
                if kind==0:
                    quat=gravity(payload[7:10])
                    if quat is None:continue
                    msg=Imu();msg.header.frame_id=camera['frame'];msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
                    msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w=quat
                    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=payload[4:7]
                    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=payload[7:10]
                    msg.orientation_covariance=payload[10:19];msg.angular_velocity_covariance=payload[19:28];msg.linear_acceleration_covariance=payload[28:37]
                    imu_pub.publish(msg)
                else:
                    stream.seek(48+payload*stride);raw=stream.read(stride);assert len(raw)==stride and struct.unpack_from('<Q',raw)[0]==original
                    datahash.update(raw)
                    ci=CameraInfo();ci.header.frame_id=camera['frame'];ci.header.stamp.sec,ci.header.stamp.nanosec=divmod(ns,10**9)
                    ci.width,ci.height=384,240;ci.k=camera['k'];ci.p=camera['p'];ci.d=camera['d'];ci.r=np.eye(3).ravel().tolist();ci.distortion_model=camera['distortion_model']
                    rgb=Image();rgb.header=ci.header;rgb.width,rgb.height,rgb.encoding,rgb.step=384,240,'bgr8',384*3;rgb.data=array.array('B',raw[8:8+384*240*3])
                    depth=Image();depth.header=ci.header;depth.width,depth.height,depth.encoding,depth.step=384,240,'32FC1',384*4;depth.data=array.array('B',raw[8+384*240*3:])
                    msg=RGBDImage();msg.header=ci.header;msg.rgb_camera_info=ci;msg.depth_camera_info=ci;msg.rgb=rgb;msg.depth=depth
                    phase='normal';source_elapsed=(original-first)*1e-9
                    if not args.no_controlled_faults and short_start<=source_elapsed<short_start+args.short_blank_s:
                        phase='blank_rgb_depth_preserved';rgb.data=array.array('B',bytes(384*240*3))
                    if not args.no_controlled_faults and long_start<=source_elapsed<long_start+args.long_blank_s:
                        phase='blank_rgb_and_depth';rgb.data=array.array('B',bytes(384*240*3));depth.data=array.array('B',bytes(384*240*4))
                    if args.blank_cut_s and ((cycle_index>0 and original-first<args.blank_cut_s*1e9) or (cycle_index+1<args.cycles and last-original<args.blank_cut_s*1e9)):
                        phase='controlled_zero_texture_video_cut';rgb.data=array.array('B',bytes(384*240*3))
                    phases[ns]=phase;msg.rgb=rgb;msg.depth=depth
                    publication=dict(ns=ns,cycle=cycle_index,original_ns=original,source_frame=payload,phase=phase,
                        rgb_sha256=hashlib.sha256(rgb.data).hexdigest(),depth_sha256=hashlib.sha256(depth.data).hexdigest())
                    published_frames.append(publication)
                    if phase!='normal':faults.append(dict(publication,kind=phase));event('injected_recorded_input_fault',publication)
                    rgbd_pub.publish(msg);published.append(ns);published_set.add(ns);timing.append((time.monotonic_ns()-target)*1e-9)
                    if logical>=next_map:
                        next_map=logical+1_000_000_000;planned.append(ns)
                        if pending is not None:dropped.append(dict(ns=pending[0],reason='superseded'))
                        pending=(ns,rgb,depth,ci,time.monotonic_ns())
                rclpy.spin_once(node,timeout_sec=0);flush_map();health()
        until=time.monotonic()+3
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005);flush_map();health()
        sent={row['ns']:row for row in map_sent};got={int(row['receipt']['header_stamp_ns']):row for row in accepted}
        assert set(got)<=set(sent),'Core admitted an image not actually published for mapping'
        for ns,row in got.items():assert row['receipt']['rgb_sha256']==sent[ns]['rgb_sha256'],'Accepted payload differs'
        valid_stamps=[ns for ns in odoms if ns in published_set and valid(ns)];steady=[ns for ns in valid_stamps if ns>=published[0]+5_000_000_000]
        elapsed=(published[-1]-published[0])*1e-9
        result.update(published=len(published),source_elapsed_s=elapsed,valid_unique_output=len(valid_stamps),valid_hz=len(steady)/(elapsed-5),
            input_payload_sha256=datahash.hexdigest(),publish_lateness_s=describe(timing),source_age_s=describe((odoms[ns]['arrival_ns']-ns)*1e-9 for ns in steady),
            map_planned=len(planned),map_published=len(sent),map_accepted=len(got),map_ack_dropped=len(dropped),
            map_not_admitted=sorted(set(sent)-set(got)),map_accept_age_s=describe((row['arrival_ns']-ns)*1e-9 for ns,row in got.items()),
            graph_messages=len(graphs),graph_display_messages=len(markers),actual_native_bindings=bindings,source_stamp_preserved=True,
            replay_timing=timing_evidence(describe(timing)))
        # Compatibility field retains its old Boolean result, with explicit
        # scope. It has never measured the live ZED sensor's real-time behavior.
        result['source_realtime_valid']=result['replay_timing']['schedule_budget_met']
        result['source_realtime_valid_scope']='deprecated alias for replay_timing.schedule_budget_met; not live sensor proof'
        result.update(continuity_audit=audit(dict(continuity=continuity,graphs=graphs,info=list(infos.values()),clouds=clouds,faults=faults)),
            cloud_messages=len(clouds),actual_vo_camera_dimensions=sorted({tuple(dim) for row in infos.values() for dim in row['camera_dimensions']}))
        result.update(faulted_images=len(faults),lost_unique_infos=sum(row['lost'] for row in infos.values()),epochs=sorted({int(row['reset_epoch']) for row in states}),
            original_source_repeated=args.cycles>1,physical_motion_at_video_cuts=False)
        result['source_age_diagnostics']=dict(
            full_odom_info_age_s=describe((r['arrival_ns']-r['ns'])*1e-9 for r in infos.values()),
            continuity_status_age_s=describe((r['arrival_ns']-int(r['status']['source_stamp_ns']))*1e-9 for r in continuity if int(r['status'].get('source_stamp_ns',0))>0),
            odom_info_age_over_300ms=sum(r['arrival_ns']-r['ns']>300_000_000 for r in infos.values()),
            source_clock_policy='fixed translation of original source clock; never slowed or restamped to satisfy freshness')
        result['bootstrap_timing']=dict(
            first_public_graph_s=min(((g['ns']-published[0])*1e-9 for g in graphs if g['node_ids']),default=None),
            first_registered_write_allowed_s=min(((int(r['status']['source_stamp_ns'])-published[0])*1e-9 for r in continuity if r['status'].get('registered') and r['status'].get('main_map_write_allowed')),default=None))
        result['public_workload_exercised']=bool(accepted and graphs and markers)
        if not result['public_workload_exercised']:result['verification_note']='not_proven: insufficient accepted main-map/graph/display workload'
        assert all(role in bindings for role in ('odom','slam')),'Native binding proof missing'
    except BaseException as exc:result['error']=repr(exc);raise
    finally:
        result['exit_codes']={}
        # Consumers close while authoritative source participants remain alive.
        for role in ('tag','submap_display','display','slam','odom'):
            p=children.get(role)
            if p is None:continue
            def signal_owned(signum):
                if p.poll() is not None:return
                owned=child_identities[role]
                actual=Path('/proc',str(p.pid),'stat').read_text().rsplit(')',1)[1].split()[19]
                assert owned['pid']==p.pid and str(owned['start_ticks'])==actual,'Owned-child PID identity changed'
                assert os.getpgid(p.pid)==p.pid,'Owned process group identity changed'
                os.killpg(p.pid,signum)
            signal_owned(signal.SIGINT)
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:signal_owned(signal.SIGKILL);p.wait(timeout=5)
            result['exit_codes'][role]=p.returncode
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if (folder/'tag/RESULT.json').exists():result['tag']=json.loads((folder/'tag/RESULT.json').read_text())
        evidence=dict(odometry=list(odoms.values()),info=list(infos.values()),accepted=accepted,graphs=graphs,markers=markers,states=states,
            continuity=continuity,clouds=clouds,published_frames=published_frames,
            registries=registries,submap_graphs=submap_graphs,submap_clouds=submap_clouds,submap_markers=submap_markers,
            tag_states=tag_states,main_tag_markers=main_tag_markers,submap_tf_edges=submap_tf_edges,
            expected_session_token_sha256=hashlib.sha256(token.encode()).hexdigest(),
            resources=samples,published=published,planned=planned,map_sent=map_sent,dropped=dropped,publish_lateness_s=timing,faults=faults)
        reset_lines=[line for line in (folder/'odom.log').read_text(errors='replace').splitlines() if 'automatically reset' in line.lower()] if (folder/'odom.log').exists() else []
        evidence['native_reset_log_lines']=reset_lines
        write_atomic(folder/'samples.json',evidence)
        result['continuity_audit']=audit(evidence)
        result['submap_workload']=dict(registries=len(registries),graphs=len(submap_graphs),clouds=len(submap_clouds),
            marker_messages=len(submap_markers),tag_states=len(tag_states),main_tag_marker_messages=len(main_tag_markers),
            rviz_started=False,rviz_config_manifest=str(folder/'submap_views/submap_views.json'))
        event_log.close()
        write_atomic(folder/'result.json',result);print(json.dumps(result),flush=True)
        write_atomic(folder/'progress.json',dict(phase='terminal',exit_codes=result['exit_codes'],error=result.get('error'),published=len(published),map_accepted=len(accepted)))
    assert all(code==0 for code in result['exit_codes'].values()),result['exit_codes']
    if args.tag_enabled:
        result['tag_detection_observed']=result.get('tag',{}).get('detected_images',0)>0
        assert result.get('tag',{}).get('state')=='closed'
    assert file_hash(receipt_path)==report['runtime_receipt_sha256'],'Runtime receipt changed during replay'
    return result

try:
    cases=[('latest_one_pending',True)] if args.latest_only else [('legacy_drop_busy',False),('latest_one_pending',True)]
    for label,enabled in cases if args.order=='ab' else reversed(cases):
        report['phase']=label;write_atomic(out/'result.json',report)
        report['cases'][label]=run_case(label,enabled);write_atomic(out/'result.json',report)
    if args.latest_only:
        case=report['cases']['latest_one_pending']
        report.update(phase='measured',source_realtime_valid=case['source_realtime_valid'],comparison=False,
            replay_timing=case['replay_timing'],live_sensor_realtime_validated=False)
    else:
        a,b=report['cases']['legacy_drop_busy'],report['cases']['latest_one_pending']
        assert a['input_payload_sha256']==b['input_payload_sha256']
        report.update(phase='measured',source_realtime_valid=a['source_realtime_valid'] and b['source_realtime_valid'],valid_rate_ratio=b['valid_hz']/a['valid_hz'],
            replay_timing={name:value['replay_timing'] for name,value in report['cases'].items()},live_sensor_realtime_validated=False)
except BaseException as exc:report.update(phase='failed',error=repr(exc));raise
finally:
    assert source_identity=={p.name:(p.stat().st_size,p.stat().st_mtime_ns) for p in SOURCE.iterdir() if p.name in source_identity}
    write_atomic(out/'result.json',report);write_atomic(TASK/'recorded_replay_latest.json',report)
