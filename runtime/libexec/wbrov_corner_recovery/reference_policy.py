"""Stable per-Tag map reference first; recent views are geometric fallbacks.

No pose averaging, fitted bridge, lowered admission limit, or new trust source.
An epoch is eligible only after its Core commit (or the original epoch zero).
Within it, only nodes still present in Core's authenticated graph are usable;
a current reanchor can exclude an older prefix of that same epoch.
"""
from measured_bridge import best_old_triplet,matrix,propose,propose_depth_candidate

def reference_readiness(live):
    """Describe retained authenticated main-map references, never grant recovery."""
    groups={}
    for (epoch,node,tag),row in live.rows.items():
        if epoch!=0 and epoch not in live.committed_epochs:continue
        if not live.graph.has_pose_for_receipt(row['binding']['accepted_image_receipt']):continue
        pose,_=live.graph.pose_for_receipt(row['binding']['accepted_image_receipt'])
        row['body']=matrix(pose)
        groups.setdefault((tag,epoch),[]).append(row)
    tags=[]
    for tag in sorted({tag for tag,epoch in groups}):
        sets=[(epoch,rows,best_old_triplet(rows)) for (t,epoch),rows in groups.items() if t==tag]
        ready=[(epoch,rows,ref) for epoch,rows,ref in sets if ref is not None]
        chosen=min(ready,key=lambda x:x[0]) if ready else max(sets,key=lambda x:len(x[1]))
        epoch,rows,ref=chosen
        tags.append(dict(tag_id=tag,reference_epoch=epoch,distinct_saved_nodes=len({r['id'] for r in rows}),
                         independent_consistent_triplet=ref is not None,
                         selected_node_ids=[r['id'] for r in ref[0]] if ref else [],
                         required_independent_references=3,
                         status='reference_ready' if ref else 'insufficient_independent_reference',
                         recovery_authorized=False))
    return tags

def historical_options(live,tag,epoch):
    groups={}
    for (e,i,t),row in live.rows.items():
        if t==tag and e<epoch and (e==0 or e in live.committed_epochs) and live.graph.has_pose_for_receipt(row['binding']['accepted_image_receipt']):
            groups.setdefault(e,[]).append(row)
    def get(e):
        rows=groups[e]
        for row in rows:
            pose,_=live.graph.pose_for_receipt(row['binding']['accepted_image_receipt'])
            row['body']=matrix(pose)
        ref=best_old_triplet(rows)
        return dict(epoch=e,reference=ref,observations=len(rows)) if ref else None
    options=[]
    # The first usable reference is not displaced merely by another reset.
    for e in sorted(groups):
        option=get(e)
        if option is not None:options.append(option);break
    if options:
        for e in sorted(groups,reverse=True):
            if e==options[0]['epoch']:break
            option=get(e)
            if option is not None:options.append(option);break
    return options

def choose_candidate(options,current,on_attempt):
    for rank,option in enumerate(options):
        try:candidate,reason=propose(option['reference'],current)
        except ValueError as exc:candidate=None;reason=str(exc)
        if candidate is None and reason in ('three_view_alignment_disagreement_or_baseline','multiple_consistent_planar_branches'):
            try:candidate,reason=propose_depth_candidate(option['reference'],current)
            except ValueError as exc:candidate=None;reason=str(exc)
        if candidate is not None:
            candidate=dict(candidate,reference_source_epoch=option['epoch'],reference_preference=rank)
        on_attempt(option,rank,candidate,reason)
        if candidate is not None:return candidate
    return None
