"""Pure, conservative correlation of passive submap/Tag replay observations."""
import math
import re


def result(checked, violations, **details):
    return dict(status='violation' if violations else 'observed_consistent' if checked else 'not_proven',
                checked=checked, violations=violations, **details)


def submap_audit(samples):
    registries=samples.get('registries',[])
    statuses=[r['status'] for r in samples.get('continuity',[])]
    expected=samples.get('expected_session_token_sha256')
    valid=[];frame_errors=[];registry_nodes={};by_revision={};by_stamp={}
    for row in registries:
        registry=row['registry'];session=registry.get('session_token_sha256','')
        error=[];nodes={};keys=set()
        if registry.get('schema')!='wbrov.submaps.v1' or not re.fullmatch('[0-9a-f]{64}',session):error.append('schema/session')
        if expected is not None and session!=expected:error.append('wrong replay session')
        if registry.get('session_short_hash')!=session[:16]:error.append('session short hash')
        for group in registry.get('submaps',[]):
            key=group.get('key');identity=f"m{group.get('map_id')}_e{group.get('reset_epoch')}_c{group.get('component_id')}"
            if key!=identity or key in keys:error.append('duplicate or invalid key')
            keys.add(key)
            frame=registry.get('main_frame_id') if group.get('registered') else f'wbrov_submap_{session[:16]}_{key}'
            if group.get('frame_id')!=frame:error.append(f'{key}: wrong local/main frame')
            if not group.get('registered') and frame in ('map','odom','base_link'):error.append(f'{key}: assumed global frame')
            for node in group.get('nodes',[]):
                ident=node['id']
                if ident in nodes:error.append(f'node {ident} occurs in several submaps')
                nodes[ident]=(group,node)
        if error:frame_errors.append(dict(revision=registry.get('revision'),reasons=error));continue
        valid.append(row);by_revision[(session,registry['revision'])]=registry
        by_stamp.setdefault(int(registry['source_stamp_ns']),[]).append(registry)
        registry_nodes[(session,registry['revision'])]=nodes
    healthy_growth=[];previous={};active=[]
    for row in valid:
        registry=row['registry'];stamp=int(registry['source_stamp_ns']);session=registry['session_token_sha256']
        exact=[s for s in statuses if int(s.get('source_stamp_ns',0))==stamp]
        for group in registry['submaps']:
            key=(session,group['key']);ids={n['id'] for n in group['nodes']}
            old=previous.get(key)
            healthy=any(s.get('local_tracking_healthy') and s.get('local_submap_write_allowed') and int(s.get('epoch',-1))==group['reset_epoch'] for s in exact)
            if not group['registered'] and group['active'] and group.get('status')=='local_tracking':
                active.append((stamp,group))
                if old is not None and ids-old and healthy:
                    healthy_growth.append(dict(ns=stamp,key=group['key'],new_node_ids=sorted(ids-old)))
            previous[key]=ids
    membership_errors=[];membership_checked=0;unmatched_main=0
    for graph in samples.get('graphs',[]):
        if not graph['node_ids']:continue
        matches=by_stamp.get(int(graph['ns']),[])
        if not matches:unmatched_main+=1;continue
        # Exact-source registry only: a later successful registration cannot
        # retrospectively authorize an earlier public graph publication.
        allowed_sets=[{n['id'] for g in reg['submaps'] if g['registered'] for n in g['nodes']} for reg in matches]
        membership_checked+=1
        if not any(set(graph['node_ids'])<=allowed for allowed in allowed_sets):
            membership_errors.append(dict(ns=graph['ns'],node_ids=graph['node_ids'],reason='public node lacks exact-source registered membership'))
    graph_errors=[];graph_checked=0;cloud_errors=[];cloud_checked=0
    for graph in samples.get('submap_graphs',[]):
        matches=[g for reg in by_stamp.get(int(graph['ns']),[]) for g in reg['submaps'] if g['key']==graph['key']]
        if not matches:continue
        graph_checked+=1
        if not any(graph['frame_id']==g['frame_id'] and set(graph['node_ids'])=={n['id'] for n in g['nodes']} for g in matches):
            graph_errors.append(dict(ns=graph['ns'],key=graph['key'],reason='graph frame/node set differs from exact registry'))
        if any(a not in graph['node_ids'] or b not in graph['node_ids'] for a,b in graph['links'] if a>0 and b>0):
            graph_errors.append(dict(ns=graph['ns'],key=graph['key'],reason='graph edge leaves its component'))
    for cloud in samples.get('submap_clouds',[]):
        matches=[g for reg in by_stamp.get(int(cloud['ns']),[]) for g in reg['submaps'] if g['key']==cloud['key']]
        if not matches:continue
        cloud_checked+=1
        frame_matches=any(cloud['frame_id']==g['frame_id'] for g in matches)
        # Registration clears the previously latched LOCAL cloud in its original
        # frame. It carries zero points, never new map geometry. A map-frame
        # clear cannot clear an RViz fixed to that independent old frame.
        old_frame_clear=cloud.get('data_bytes')==0 and cloud.get('width')==0 and any(
            int(row['registry']['source_stamp_ns'])<=int(cloud['ns']) and any(
                g['key']==cloud['key'] and g['frame_id']==cloud['frame_id'] for g in row['registry']['submaps'])
            for row in valid)
        if not frame_matches and not old_frame_clear:cloud_errors.append(dict(ns=cloud['ns'],key=cloud['key'],reason='cloud frame differs from registry'))
    loss_separations=[];loss_errors=[]
    losses=sorted({int(r['ns']) for r in samples.get('info',[]) if r.get('lost')})
    seen_pairs=set()
    for loss in losses:
        before=[(stamp,g) for stamp,g in active if stamp<loss]
        after=[(stamp,g) for stamp,g in active if stamp>loss and any(int(n['source_stamp_ns'])>loss for n in g['nodes'])]
        if not before or not after:continue
        prior=max(before,key=lambda r:r[0]);following=min(after,key=lambda r:r[0]);pair=(prior[1]['key'],following[1]['key'])
        if pair in seen_pairs:continue
        seen_pairs.add(pair)
        row=dict(loss_ns=loss,before_key=pair[0],after_key=pair[1],before_epoch=prior[1]['reset_epoch'],after_epoch=following[1]['reset_epoch'])
        (loss_separations if row['after_epoch']>row['before_epoch'] and pair[0]!=pair[1] else loss_errors).append(row)
    infos={int(r['ns']):r for r in samples.get('info',[]) if 'ns' in r};bad_writes=[];quality_checked=0
    for row in samples.get('accepted',[]):
        receipt=row['receipt'];stamp=int(receipt['header_stamp_ns']);info=infos.get(stamp)
        if info is None:continue
        quality_checked+=1
        if info.get('lost'):bad_writes.append(dict(ns=stamp,node_id=receipt.get('node_id'),reason='Core receipt on exact native lost frame'))
    # A status received after process may already be observing a proposal. Only
    # the interior of two equally blocked local status samples is asserted.
    for a,b in zip(statuses,statuses[1:]):
        begin,end=int(a.get('source_stamp_ns',0)),int(b.get('source_stamp_ns',0))
        if not (0<begin<end and a.get('epoch')==b.get('epoch') and a.get('generation')==b.get('generation')):continue
        if any(s.get('main_map_write_allowed',True) or s.get('local_submap_write_allowed',True) for s in (a,b)):continue
        for row in samples.get('accepted',[]):
            receipt=row['receipt'];stamp=int(receipt['header_stamp_ns'])
            if begin<stamp<end:bad_writes.append(dict(ns=stamp,node_id=receipt.get('node_id'),reason='receipt inside measured fully-blocked source interval'))
    tag_errors=[];tag_checked=0;placed=0;unplaced=0;main_tag_centers=[]
    for row in samples.get('tag_states',[]):
        state=row['state']
        if not state.get('visualization_valid'):continue
        identity=(state.get('session_token_sha256'),state.get('registry_revision'))
        registry=by_revision.get(identity)
        if registry is None:
            # Missing latched history means insufficient evidence, not proof of
            # a wrong placement. Explicit wrong session remains a violation.
            if expected is not None and identity[0]!=expected:tag_errors.append(dict(reason='Tag state wrong replay session'))
            continue
        tag_checked+=1;nodes=registry_nodes[identity]
        for tag in state.get('tags',[]):
            if tag.get('placement_status')=='unplaced':
                unplaced+=1
                if 'center' in tag or 'corners3d' in tag:tag_errors.append(dict(reason='unplaced Tag has invented coordinates'))
                continue
            located=nodes.get(tag.get('node_id'));placed+=1
            if located is None:tag_errors.append(dict(reason='placed Tag node absent from registry'));continue
            group,node=located
            want=(group['key'],group['map_id'],group['reset_epoch'],group['frame_id'],int(node['source_stamp_ns']),
                  'main_registered' if group['registered'] else 'local_submap')
            got=(tag.get('submap_key'),tag.get('map_id'),tag.get('reset_epoch'),tag.get('frame_id'),int(tag.get('source_stamp_ns',0)),tag.get('placement_status'))
            if want!=got:tag_errors.append(dict(reason='Tag node/session/frame/epoch/placement mismatch',node_id=tag.get('node_id')))
            coords=tag.get('center',[])+[v for p in tag.get('corners3d',[]) for v in p]
            if len(coords)!=15 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in coords):tag_errors.append(dict(reason='invalid Tag geometry'))
            if tag.get('relocalization_verified') is not False:tag_errors.append(dict(reason='visualization falsely claims recovery'))
            if group['registered'] and len(tag.get('center',[]))==3:main_tag_centers.append(tag['center'])
    marker_errors=[];marker_checked=0
    for row in samples.get('main_tag_markers',[]):
        for marker in row['markers']:
            if marker['action']!=0:continue
            marker_checked+=1
            if marker['frame_id']!='map':marker_errors.append(dict(reason='main Tag marker in non-main frame'))
            if marker['namespace']=='tag_estimated_center' and not any(sum((a-b)**2 for a,b in zip(marker['position'],p))<1e-8 for p in main_tag_centers):
                marker_errors.append(dict(reason='main Tag center has no observed registered Tag placement'))
    for edge in samples.get('submap_tf_edges',[]):frame_errors.append(dict(reason='unexpected TF touches independent submap frame',edge=edge))
    return dict(
        local_healthy_submap_growth=dict(status='observed' if healthy_growth else 'not_proven',events=healthy_growth),
        main_graph_registered_membership=result(membership_checked,membership_errors,unmatched_source_graphs=unmatched_main),
        independent_submap_frames=result(len(valid),frame_errors,forbidden_tf_edges=samples.get('submap_tf_edges',[])),
        submap_graph_registry_match=result(graph_checked,graph_errors),submap_cloud_registry_frame=result(cloud_checked,cloud_errors),
        loss_creates_separate_local_epoch=result(len(loss_separations),loss_errors,events=loss_separations),
        untrusted_frame_write_gate=result(quality_checked,bad_writes),
        tag_registry_identity=result(tag_checked,tag_errors,placed=placed,unplaced=unplaced),
        main_tag_markers_registered_only=result(marker_checked,marker_errors),
        recovery_claim='Use post-established-main loss, pause, registered resumption AND new public nodes; local growth/Tag display alone is not recovery.',
        limitations=['No active database inspection or physical ground truth. Exact-source registry correlation can be incomplete across asynchronous publishers.',
                    'Tag coordinates are receipt-bound planar estimates, not independent depth verification or successful relocalization.',
                    'Marker-to-Tag geometry is checked against recorded history; this does not certify every asynchronous display transition.'])
