"""Pure evidence audit for the recorded native replay, with explicit limits."""
from bisect import bisect_right
from submap_replay_audit import submap_audit


def graph_components(ids, links):
    nodes={int(n) for n in ids if int(n)>0}
    parent={n:n for n in nodes}
    def find(n):
        while parent[n]!=n:
            parent[n]=parent[parent[n]];n=parent[n]
        return n
    for a,b in links:
        if a in nodes and b in nodes:parent[find(a)]=find(b)
    return len({find(n) for n in nodes})


def audit(samples):
    statuses=sorted(samples.get('continuity',[]),key=lambda row:row['arrival_steady_ns'])
    graphs=sorted(samples.get('graphs',[]),key=lambda row:(row['ns'],row['arrival_steady_ns']))
    info=samples.get('info',[])
    clouds=sorted(samples.get('clouds',[]),key=lambda row:(row['ns'],row['arrival_steady_ns']))
    # Only source-time intervals whose two measured endpoints are in the same
    # blocked generation/epoch are asserted. Transition gaps stay unproven.
    spans=[]
    for a,b in zip(statuses,statuses[1:]):
        x,y=a['status'],b['status']
        if (not x.get('main_map_write_allowed',True) and not y.get('main_map_write_allowed',True)
            and x.get('generation')==y.get('generation') and x.get('epoch')==y.get('epoch')
            and 0<int(x.get('source_stamp_ns',0))<int(y.get('source_stamp_ns',0))):
            spans.append((int(x['source_stamp_ns']),int(y['source_stamp_ns']),
                          bool(x.get('observing_registered_map') or y.get('observing_registered_map') or
                               x.get('recovery_commit_pending') or y.get('recovery_commit_pending'))))
    graph_ns=[r['ns'] for r in graphs]
    growth=[];covered=0;observation_spans=0;blocked_graph_messages=0
    for begin,end,observing in spans:
        index=bisect_right(graph_ns,begin)-1
        if index<0 or not graphs[index]['node_ids']:continue
        covered+=1;observation_spans+=int(observing)
        baseline=set(graphs[index]['node_ids'])
        for g in graphs[index+1:]:
            if g['ns']>end:break
            if g['ns']<=begin:continue
            blocked_graph_messages+=1
            added=set(g['node_ids'])-baseline
            if added:growth.append({'interval_start_ns':begin,'interval_end_ns':end,
                                   'graph_stamp_ns':g['ns'],'new_node_ids':sorted(added)})
    public_nonempty=[g for g in graphs if g['node_ids']]
    second=[{'ns':g['ns'],'components':g['components'],'node_ids':g['node_ids']}
            for g in public_nonempty if g['components']>1]
    depth=[r for r in info if not r['lost'] and r.get('inliers',0)==0 and
           r.get('icp_correspondences',0)>=80 and r.get('icp_inliers_ratio',0)>=.55 and
           r.get('icp_structural_complexity',0)>=1e-4 and r.get('icp_structural_distribution',0)>=.5]
    short_depth=[r for r in depth if r.get('fault_phase')=='blank_rgb_depth_preserved']
    long_loss=[r for r in info if r['lost'] and r.get('fault_phase')=='blank_rgb_and_depth']
    epochs=sorted({r['status']['epoch'] for r in statuses if 'epoch' in r['status']})
    pause_seen=any(r['status'].get('state')=='PAUSED' for r in statuses)
    observing=any(r['status'].get('observing_registered_map') or r['status'].get('recovery_commit_pending') for r in statuses)
    observation_complete=any('observations passed' in r['status'].get('reason','') or
                             'observed candidate committed' in r['status'].get('reason','') for r in statuses)
    resume_events=[]
    lost_stamps=sorted(r['ns'] for r in info if r['lost'])
    for lost in lost_stamps:
        before=[g for g in public_nonempty if g['ns']<lost]
        established=any(int(r['status'].get('source_stamp_ns',0))<lost and r['status'].get('registered') and
                        r['status'].get('main_map_write_allowed') for r in statuses)
        if not before or not established:continue
        baseline=max(before,key=lambda g:g['ns'])
        paused=[r['status'] for r in statuses if int(r['status'].get('source_stamp_ns',0))>=lost and
                not r['status'].get('main_map_write_allowed',True)]
        if not paused:continue
        pause_ns=min(int(s['source_stamp_ns']) for s in paused)
        resumed=[r['status'] for r in statuses if int(r['status'].get('source_stamp_ns',0))>pause_ns and
                 r['status'].get('registered') and r['status'].get('main_map_write_allowed')]
        if not resumed:continue
        resumed_ns=min(int(s['source_stamp_ns']) for s in resumed)
        grew=[g for g in public_nonempty if g['ns']>=resumed_ns and set(g['node_ids'])-set(baseline['node_ids'])]
        if grew:
            resume_events.append(dict(loss_ns=lost,pause_ns=pause_ns,resumed_ns=resumed_ns,
                main_graph_growth_ns=grew[0]['ns'],new_node_ids=sorted(set(grew[0]['node_ids'])-set(baseline['node_ids']))))
            break
    cloud_ns=[r['ns'] for r in clouds];cloud_spans=0;cloud_changes=[]
    for begin,end,_ in spans:
        index=bisect_right(cloud_ns,begin)-1
        if index<0 or not clouds[index].get('data_bytes',0):continue
        cloud_spans+=1;baseline=clouds[index]
        for c in clouds[index+1:]:
            if c['ns']>end:break
            if c['ns']<=begin:continue
            if (c['frame_id'],c['width'],c['height'],c['data_sha256'])!=(baseline['frame_id'],baseline['width'],baseline['height'],baseline['data_sha256']):
                cloud_changes.append({'interval_start_ns':begin,'interval_end_ns':end,'cloud_stamp_ns':c['ns']})
    faults=samples.get('faults',[])
    fault_bootstrap=[]
    for kind in sorted({f['kind'] for f in faults}):
        first_fault=min(f['ns'] for f in faults if f['kind']==kind)
        prior=[s['status'] for s in statuses if int(s['status'].get('source_stamp_ns',0))<first_fault]
        established=any(g['ns']<first_fault for g in public_nonempty) and any(s.get('registered') and s.get('main_map_write_allowed') for s in prior)
        fault_bootstrap.append({'kind':kind,'first_fault_ns':first_fault,'main_map_established_before_fault':established})
    return {
        'status_messages':len(statuses),'blocked_source_intervals':len(spans),
        'blocked_intervals_with_existing_public_graph':covered,
        'observation_intervals_with_existing_public_graph':observation_spans,
        'public_graph_messages_inside_blocked_intervals':blocked_graph_messages,
        'main_graph_growth_during_monitored_blocked_intervals':{
            'status':'violation' if growth else 'observed_no_growth' if covered else 'not_proven',
            'violations':growth},
        'second_disconnected_public_map':{
            'status':'violation' if second else 'not_observed' if public_nonempty else 'not_proven',
            'violations':second},
        'native_depth_fallback':{'status':'observed' if depth else 'not_proven','count':len(depth)},
        'controlled_short_depth_fallback':{'status':'observed' if short_depth else 'not_proven','count':len(short_depth)},
        'fault_bootstrap_preconditions':fault_bootstrap,
        'public_cloud_payload_during_monitored_blocked_intervals':{
            'status':'changed' if cloud_changes else 'observed_unchanged' if cloud_spans else 'not_proven',
            'intervals_with_existing_public_cloud':cloud_spans,'payload_changes':cloud_changes},
        'controlled_long_loss':{'status':'observed' if long_loss else 'not_proven','lost_frames':len(long_loss)},
        'native_reset':{'status':'observed' if len(epochs)>1 and samples.get('native_reset_log_lines') else 'not_proven',
                        'epochs':epochs,'native_reset_log_lines':samples.get('native_reset_log_lines',[])},
        'registered_map_observation':{'status':'observed' if observing else 'not_proven',
                                      'completion':'observed' if observation_complete else 'not_proven'},
        'resume_after_pause':{'status':'observed' if resume_events else 'not_proven','events':resume_events},
        'submaps_and_tags':submap_audit(samples),
        'paused_state_observed':pause_seen,
        'limitations':[
            'Monitors public trusted MapGraph/cloud outputs; does not inspect active SQLite or prove hidden internal graph state.',
            'No public growth during source-correlated blocked intervals does not prove every asynchronous transition instant.',
            'Connected public graph does not by itself prove spatially correct non-overlapping point clouds.',
            'A cloud payload change is evidence to inspect; serialization or map correction can change it without adding points.',
            'Images/depth are correlated ZED data; reverse registration and repeated frames are not independent sensor evidence.',
            'Recorded 384x240 CPU replay does not prove live camera/GPU timing or underwater geometric accuracy.',
        ],
    }
