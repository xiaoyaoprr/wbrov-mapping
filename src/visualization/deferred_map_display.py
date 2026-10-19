#!/usr/bin/env python3
"""Display only receipt-verified historical map output; never publish robot TF."""
from pathlib import Path
import argparse
import hashlib
import json
import cv2
import numpy as np
from PIL import Image as PillowImage,ImageDraw,ImageFont
from deferred_display_state import mapping_label,trajectory_data,pose_axes
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField,Image
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from geometry_msgs.msg import Point


def verified_cloud(root):
    saved=root/'save.json'
    if saved.exists():
        receipt=json.loads(saved.read_text())
        if receipt.get('complete') is not True and receipt.get('artifacts_saved') is not True:return None
        expected=receipt['sha256']['main_cloud.ply']
    else:
        receipts=sorted(root.glob('receipt-????????.json'))
        if not receipts:return None
        expected=json.loads(receipts[-1].read_text())['cloud_sha256']
    path=root/'mapping/main_cloud.ply'
    if not path.exists() or path.stat().st_size>512*1024*1024:return None
    data=path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=expected:return None # next chunk is being sealed
    boundary=data.find(b'end_header\n')
    if boundary<0 or boundary>8192:raise ValueError('invalid PLY header')
    header=data[:boundary].decode('ascii')
    if 'format binary_little_endian 1.0' not in header:raise ValueError('unsupported cloud encoding')
    count=int(next(line.split()[-1] for line in header.splitlines() if line.startswith('element vertex ')))
    pixels=data[boundary+11:]
    if len(pixels)!=count*15:raise ValueError('cloud vertex count mismatch')
    return expected,count,pixels


def verified_tags(root):
    path=root/'mapping/tags.json'
    if not path.exists():return None
    saved=root/'save.json'
    if saved.exists():expected=json.loads(saved.read_text()).get('sha256',{}).get('tags.json')
    else:
        receipts=sorted(root.glob('receipt-????????.json'))
        if not receipts:return None
        expected=json.loads(receipts[-1].read_text()).get('tags_sha256')
    data=path.read_bytes()
    if len(data)>4*1024*1024 or hashlib.sha256(data).hexdigest()!=expected:return None
    value=json.loads(data)
    if value.get('historical_only') is not True or value.get('frame_id')!='deferred_map':raise ValueError('invalid Tag provenance')
    return expected,value


class Display(Node):
    def __init__(self,root):
        super().__init__('wbrov_deferred_map_display')
        self.root=root;self.digest=None;self.tag_digest=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud=self.create_publisher(PointCloud2,'/wbrov/deferred/main_cloud',qos)
        self.status=self.create_publisher(String,'/wbrov/deferred/status',qos)
        self.label=self.create_publisher(Marker,'/wbrov/deferred/status_label',qos)
        self.tags=self.create_publisher(MarkerArray,'/wbrov/deferred/tags',qos)
        self.trajectory=self.create_publisher(MarkerArray,'/wbrov/deferred/trajectory',qos)
        self.status_image=self.create_publisher(Image,'/wbrov/deferred/status_image',qos)
        font=next((p for p in [Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')] if p.exists()),None)
        if font is None:raise RuntimeError('Chinese display font is required')
        self.font=ImageFont.truetype(str(font),27)
        self.create_timer(.5,self.update)

    def update(self):
        path=self.root/'status.json'
        if not path.exists():return
        try:
            status=json.loads(path.read_text())
            if status.get('historical_only') is not True or status.get('navigation_authorized') is not False:
                raise ValueError('output provenance does not permit historical-only display')
            found=verified_cloud(self.root)
            if found and found[0]!=self.digest:
                digest,count,pixels=found
                cloud=PointCloud2();cloud.header.frame_id='deferred_map'
                # Historical acquisition time, never now()/current navigation.
                stamp=int(status.get('verified_map_source_ns',0));cloud.header.stamp.sec=stamp//1000000000;cloud.header.stamp.nanosec=stamp%1000000000
                cloud.height=1;cloud.width=count;cloud.is_bigendian=False;cloud.is_dense=True;cloud.point_step=16;cloud.row_step=count*16
                cloud.fields=[PointField(name=n,offset=o,datatype=PointField.FLOAT32,count=1) for n,o in [('x',0),('y',4),('z',8),('rgb',12)]]
                original=np.frombuffer(pixels,dtype=np.dtype([('xyz','<f4',(3,)),('r','u1'),('g','u1'),('b','u1')]))
                packed=np.zeros(count,dtype=np.dtype([('xyz','<f4',(3,)),('rgb','<u4')]))
                packed['xyz']=original['xyz'];packed['rgb']=(original['r'].astype(np.uint32)<<16)|(original['g'].astype(np.uint32)<<8)|original['b']
                cloud.data=packed.tobytes();self.cloud.publish(cloud);self.digest=digest
                self.get_logger().info(f'verified recorded cloud published: points={count} sha256={digest}')
            self.status.publish(String(data=json.dumps(status,ensure_ascii=False)))
            tags=verified_tags(self.root)
            if tags and tags[0]!=self.tag_digest:
                array=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
                # Lift labels clear of dense wall points; the leader ends at
                # the measured centre, so a label offset cannot look like a
                # different Tag location.
                label_height=max([.35]+[float(t['position'][2])+.20 for t in tags[1]['tags'] if 'position' in t])
                for tag in tags[1]['tags']:
                    if 'position' not in tag:continue
                    marker=Marker();marker.header.frame_id='deferred_map';marker.ns='recorded_tags';marker.id=tag['tag_id']
                    marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose.orientation.w=1.
                    marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=map(float,tag['position'])
                    marker.pose.position.z=label_height;marker.scale.z=.065;marker.color.a=1.;marker.color.g=1.;marker.color.r=1.
                    # Keep the 3-D label compact. RViz's bundled font renders
                    # spaces with an excessive advance on this Jetson; the
                    # full reference count belongs in the status panel.
                    marker.text=f"Tag{tag['tag_id']}"
                    array.markers.append(marker)
                    dot=Marker();dot.header.frame_id='deferred_map';dot.ns='recorded_tag_centres';dot.id=tag['tag_id']
                    dot.type=Marker.SPHERE;dot.action=Marker.ADD;dot.pose.orientation.w=1.
                    dot.pose.position.x,dot.pose.position.y,dot.pose.position.z=map(float,tag['position'])
                    dot.scale.x=dot.scale.y=dot.scale.z=.035;dot.color.a=1.;dot.color.r=1.;dot.color.g=.8
                    array.markers.append(dot)
                    leader=Marker();leader.header.frame_id='deferred_map';leader.ns='recorded_tag_callouts';leader.id=tag['tag_id']
                    leader.type=Marker.LINE_LIST;leader.action=Marker.ADD;leader.pose.orientation.w=1.;leader.scale.x=.003
                    leader.color.a=.9;leader.color.r=1.;leader.color.g=.8
                    leader.points=[Point(x=dot.pose.position.x,y=dot.pose.position.y,z=dot.pose.position.z),
                                   Point(x=marker.pose.position.x,y=marker.pose.position.y,z=label_height-.04)]
                    array.markers.append(leader)
                self.tags.publish(array);self.tag_digest=tags[0]
                edges,latest=trajectory_data(tags[1]);trajectory=MarkerArray();trajectory.markers.append(clear)
                path=Marker();path.header.frame_id='deferred_map';path.ns='trusted_recorded_trajectory';path.id=0
                path.type=Marker.LINE_LIST;path.action=Marker.ADD;path.pose.orientation.w=1.;path.scale.x=.008
                path.color.a=1.;path.color.g=1.;path.color.b=1.
                for pair in edges:
                    for xyz in pair:path.points.append(Point(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2])))
                trajectory.markers.append(path)
                if latest:
                    for axis_id,(label,color,tip) in enumerate(pose_axes(latest)):
                        robot=Marker();robot.header.frame_id='deferred_map';robot.ns='last_verified_recorded_axes';robot.id=axis_id
                        robot.header.stamp.sec=latest['source_ns']//1000000000;robot.header.stamp.nanosec=latest['source_ns']%1000000000
                        robot.type=Marker.ARROW;robot.action=Marker.ADD;robot.pose.orientation.w=1.
                        robot.points=[Point(x=float(latest['position'][0]),y=float(latest['position'][1]),z=float(latest['position'][2])),Point(x=tip[0],y=tip[1],z=tip[2])]
                        robot.scale.x=.008;robot.scale.y=.018;robot.scale.z=.025;robot.color.a=1.
                        robot.color.r,robot.color.g,robot.color.b=color
                        trajectory.markers.append(robot)
                        text=Marker();text.header=robot.header;text.ns='last_verified_recorded_axis_labels';text.id=axis_id
                        text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD;text.pose.orientation.w=1.
                        text.pose.position.x,text.pose.position.y,text.pose.position.z=tip;text.scale.z=.035
                        text.color=robot.color;text.text=label;trajectory.markers.append(text)
                self.trajectory.publish(trajectory)
                self.get_logger().info(f"verified overlays published: tags={len(tags[1]['tags'])} trajectory_edges={len(edges)} recorded_pose={latest['node_id'] if latest else 0}")
            marker=Marker();marker.header.frame_id='deferred_map';marker.ns='historical_mapping';marker.id=0
            marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.pose.position.z=.5
            marker.scale.z=.035;marker.color.r=1.;marker.color.g=.8;marker.color.b=.2;marker.color.a=1.
            lag=status.get('backlog_seconds');lag='--' if lag is None else f'{lag:.1f}s'
            marker.text=f"Recorded map | backlog {lag}\n{status['state']} | source time, not current robot pose"
            self.label.publish(marker)
            canvas=PillowImage.new('RGB',(680,270),(25,27,30));draw=ImageDraw.Draw(canvas)
            mapping=status.get('mapping',{})
            title,color=mapping_label(status)
            capture='采集已关闭' if status.get('recording_state')=='closed' else '持续采集'
            entry_path=self.root.parent/'entry_status.json'
            if entry_path.exists():
                entry=json.loads(entry_path.read_text())
                if entry.get('phase')=='draining':
                    capture='采集已停止，正在保存'
                    if entry.get('errors'):title,color='采集中断，正在保存封存数据',(255,160,80)
                elif entry.get('phase')=='capturing' and entry.get('seconds_without_new_seal',0)>3:
                    capture=f"采集/写盘等待 {entry['seconds_without_new_seal']:.0f}s"
            lines=[title,f"{capture}  |  待处理：{lag}",
                   f"主图：{mapping.get('main_nodes',0)} 个节点  |  恢复：{mapping.get('recoveries',0)} 次"]
            for tag in (tags[1]['tags'] if tags else [])[:2]:
                lines.append(f"Tag {tag['tag_id']}：{tag['independent_references']}/3 组参考，{tag['distinct_main_nodes']} 个节点")
            lines+=['青线：可信历史轨迹；黄点：Tag 位置','红X/绿Y/蓝Z：已处理位姿，非实时位姿']
            for i,line in enumerate(lines):draw.text((12,5+37*i),line,font=self.font,fill=color if i==0 else (235,235,235))
            message=Image();message.header.stamp=self.get_clock().now().to_msg();message.height=270;message.width=680
            message.encoding='rgb8';message.step=2040;message.data=canvas.tobytes();self.status_image.publish(message)
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.get_logger().warning(str(error),throttle_duration_sec=5.)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    rclpy.init();node=Display(args.output.resolve(strict=True))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
