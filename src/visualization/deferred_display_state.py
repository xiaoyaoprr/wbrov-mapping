"""Presentation policy for historical mapping, independent of ROS and clocks."""
import math


def mapping_label(status):
    stage=status.get('state');mapping=status.get('mapping',{})
    if stage=='failed':return '处理失败，已保留输入',(255,90,90)
    if stage=='closed':return '已保存',(100,220,160)
    if stage=='interrupted_saved':return '采集中断，已保存封存数据',(255,160,80)
    if mapping.get('continuity_state')=='RECOVERING':return '恢复核验中',(255,210,80)
    if mapping.get('main_map_write_allowed') is True:return '建图中',(100,240,130)
    if mapping.get('continuity_state') in ('PAUSED','DEGRADED','NORMAL'):return '主图暂停，继续处理观测',(255,160,80)
    return '等待建图状态',(210,210,210)


def trajectory_data(payload):
    nodes={}
    for node in payload.get('trajectory',[]):
        p=node['position'];q=node['orientation_xyzw'];identity=node['node_id']
        if type(identity) is not int or identity<=0 or identity in nodes:raise ValueError('invalid trajectory node')
        if len(p)!=3 or len(q)!=4 or not all(math.isfinite(x) for x in p+q):raise ValueError('invalid trajectory geometry')
        if abs(sum(x*x for x in q)-1.)>.01:raise ValueError('invalid trajectory quaternion')
        if type(node['source_ns']) is not int or node['source_ns']<=0:raise ValueError('invalid trajectory source time')
        nodes[identity]=node
    edges=[]
    for a,b in payload.get('trajectory_edges',[]):
        if a not in nodes or b not in nodes or a==b or nodes[a]['epoch']!=nodes[b]['epoch']:
            raise ValueError('trajectory must not connect missing or unaligned epochs')
        edges.append((nodes[a]['position'],nodes[b]['position']))
    latest=max(nodes.values(),key=lambda n:n['source_ns']) if nodes else None
    return edges,latest


def pose_axes(node,length=.18):
    """Body X/Y/Z endpoints in the recorded map frame, using its full rotation."""
    p=node['position'];q=node['orientation_xyzw']
    if len(p)!=3 or len(q)!=4 or not all(math.isfinite(v) for v in p+q):raise ValueError('invalid axis pose')
    norm=sum(v*v for v in q)
    if abs(norm-1.)>.01:raise ValueError('invalid axis quaternion')
    x,y,z,w=[v/math.sqrt(norm) for v in q]
    columns=[(1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)),
             (2*(x*y-w*z),1-2*(x*x+z*z),2*(y*z+w*x)),
             (2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y))]
    return [(label,color,[float(p[i])+length*direction[i] for i in range(3)])
            for label,color,direction in zip('XYZ',[(1.,0.,0.),(0.,1.,0.),(0.,.35,1.)],columns)]
