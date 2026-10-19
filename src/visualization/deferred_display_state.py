"""Presentation policy for historical mapping, independent of ROS and clocks."""
import math


def recovery_lines(mapping, verified_tags):
    """Explain recorded evidence; absence of a Tag never means visual failure.

    None is an unverified/missing export, distinct from a verified empty list.
    Observation counts are only displayed for their own recovery attempt.
    """
    lines=[]
    if verified_tags is None:
        lines.append('主图 Tag 参考：等待核验；仍可尝试视觉恢复')
    else:
        tags=verified_tags.get('tags',[])
        ready=sum(t.get('independent_references',0)>=t.get('required_references',3) for t in tags)
        lines.append(f'主图 Tag：{len(tags)} 个，参考齐备：{ready} 个')
        if not tags:lines.append('主图无 Tag，仍可用旧关键帧尝试恢复')
        for tag in tags:
            lines.append(f"Tag {tag['tag_id']}：{tag.get('independent_references',0)}/3 组有效参考，{tag.get('distinct_main_nodes',0)} 个节点")
    detected=mapping.get('detected_tags_in_processed_frame')
    if detected is not None:
        lines.append(f'已处理帧检测到 Tag：{detected} 个（不等于已绑定）')
    route=mapping.get('recovery_route')
    if mapping.get('main_map_write_allowed') is not True:
        if route=='visual_keyframes':lines.append('最近恢复通道：旧关键帧，不要求 Tag')
        elif route=='tag_depth_corners':lines.append('Tag 几何核验中，尚未授权主图写入')
        d=mapping.get('observation_diagnostic',{})
        if d.get('source_ns',0)>0 and d.get('source_ns')==mapping.get('recovery_attempt_source_ns'):
            lines.append(f"最近视觉核验：参考 {d.get('references',0)}，测试 {d.get('tested',0)}，配上 {d.get('matched',0)}")
            if d.get('prediction_rejected',0) or d.get('agreement_rejected',0):
                lines.append('配准结果不一致，继续保留原图')
            elif d.get('baseline_rejected',0):lines.append('旧参考视角不够独立，需换已建区域')
            elif d.get('raw_rejected',0):lines.append('旧画面匹配不足，请查看已建区域纹理')
            elif not d.get('nearby',0):lines.append('当前预测附近无参考，不能据此确定位置')
        quality=mapping.get('quality') or {}
        if quality.get('input_present') is False:lines.append('当前帧输入缺失，等待有效观测')
        elif quality.get('usable') is False:
            if quality.get('lost'):lines.append('当前帧跟踪丢失，主图保持不变')
            elif not quality.get('finite_motion',True):lines.append('当前运动估计无效，暂停写图')
            elif not quality.get('dimensions_valid',True):lines.append('图像尺寸/模型不一致，暂停写图')
            elif not quality.get('covariance_valid',True):lines.append('位姿不确定度不合格，暂停写图')
            else:lines.append('图像/深度几何支撑不足，暂停写图')
    return lines


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
