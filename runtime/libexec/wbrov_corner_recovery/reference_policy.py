"""Stable per-Tag map reference first; recent views are geometric fallbacks.

No pose averaging, fitted bridge, lowered admission limit, or new trust source.
An epoch is eligible only after its Core commit (or the original epoch zero).
"""
from measured_bridge import best_old_triplet,matrix,propose

def historical_options(live,tag,epoch):
    groups={}
    for (e,i,t),row in live.rows.items():
        if t==tag and e<epoch and (e==0 or e in live.committed_epochs):
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
        if candidate is not None:
            candidate=dict(candidate,reference_source_epoch=option['epoch'],reference_preference=rank)
        on_attempt(option,rank,candidate,reason)
        if candidate is not None:return candidate
    return None
