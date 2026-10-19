"""Retain stable old-map evidence and a recent fallback for each Tag.

Never edits Core keyframes, a saved map, measurements, or source identities.
The complete observation/transaction log remains on disk. Bounds scale with
distinct Tags, not completed frames or repeated tracking epochs.
"""
from collections import defaultdict

def ranked_old(rows):
    from measured_bridge import reference_reservoir
    return reference_reservoir(rows)

def current_rows(rows):
    if not rows:return []
    newest=max(rows,key=lambda r:r['stamp']);now=newest['stamp']
    # Exactly the reservoir used by best_old_triplet, plus every view which
    # propose() can currently use and the newest timestamp even without PnP.
    selected=ranked_old(rows)+[r for r in rows if r['branches'] and now-r['stamp']<=8*10**9][-12:]+[newest]
    ids={r['id'] for r in selected}
    return [r for r in rows if r['id'] in ids]

def compact(live,pending_ready):
    from measured_bridge import best_old_triplet,matrix
    consumer=live.consumer;graph=live.graph
    if graph.current is None:return
    epoch=graph.current['state']['reset_epoch']
    grouped=defaultdict(list)
    for key,row in live.rows.items():
        if graph.has_pose_for_receipt(row['binding']['accepted_image_receipt']):
            grouped[(key[0],key[2])].append(row)
    keep=set()
    for (e,tag),rows in grouped.items():
        if e==epoch:keep.update((e,r['id'],tag) for r in current_rows(rows))
    # Preserve the first usable trusted reference instead of replacing it on
    # every reset. Also keep the newest reservoir (late deliveries may extend
    # it) and newest usable fallback: at most three 24-view sets per Tag.
    # Cache that decision until an epoch transition or late old observation.
    if live.history_dirty or live.history_epoch!=epoch:
        historic=set();tags={tag for e,tag in grouped if e<epoch}
        for tag in tags:
            choices=sorted((e for e,t in grouped if t==tag and e<epoch and
                            (e==0 or e in live.committed_epochs)),reverse=True)
            for index,e in enumerate(choices):
                rows=ranked_old(grouped[(e,tag)])
                for row in rows:
                    pose,_=graph.pose_for_receipt(row['binding']['accepted_image_receipt']);row['body']=matrix(pose)
                if index==0:historic.update((e,r['id'],tag) for r in rows)
                if best_old_triplet(rows) is not None:
                    historic.update((e,r['id'],tag) for r in rows);break
            for e in reversed(choices):
                rows=ranked_old(grouped[(e,tag)])
                for row in rows:
                    pose,_=graph.pose_for_receipt(row['binding']['accepted_image_receipt']);row['body']=matrix(pose)
                if best_old_triplet(rows) is not None:
                    historic.update((e,r['id'],tag) for r in rows);break
        live.historic_keys=historic;live.history_epoch=epoch;live.history_dirty=False
    keep.update(live.historic_keys)
    # A pending transaction or deferred retry pins all its original receipts.
    pinned=set()
    for p in ([live.pending] if live.pending else [])+[d['proposal'] for d in live.deferred]:
        for edge in p['evidence']:pinned.update((edge['from_id'],edge['to_id']))
    for k,reference in list(live.references.items()):
        if k[0]!=epoch or any(not graph.has_pose_for_receipt(r['binding']['accepted_image_receipt']) for r in reference[0]):del live.references[k]
        else:pinned.update(r['id'] for r in reference[0])
    keep.update(k for k in live.rows if k[1] in pinned)
    old_counts=(len(live.rows),len(consumer.bindings),len(consumer.receipts))
    live.rows={k:r for k,r in live.rows.items() if k in keep}
    binding_keys={(i,t) for e,i,t in keep}
    # Don't evict a join whose ready callback has not finished, including Tags
    # which have bound but have not yet been inserted into live.rows.
    for ns in pending_ready:
        if ns in consumer.receipts:pinned.add(consumer.receipts[ns]['receipt']['node_id'])
    for ns in consumer.pending:
        if ns in consumer.receipts:pinned.add(consumer.receipts[ns]['receipt']['node_id'])
    for key in list(consumer.bindings):
        if key not in binding_keys and key[0] not in pinned:del consumer.bindings[key]
    needed_nodes={i for i,t in consumer.bindings}|pinned
    # Receipts awaiting detector/graph work remain bounded by their existing
    # 4096 admission cap. Only explicitly completed frames can be reclaimed.
    for ns,entry in list(consumer.receipts.items()):
        node=entry['receipt']['node_id']
        if node not in needed_nodes and ns in live.completed_for_retention:
            del consumer.receipts[ns];consumer.node_receipts.pop(node,None)
    live.completed_for_retention.intersection_update(consumer.receipts)
    live.processed.intersection_update(pending_ready)
    graph.compact(consumer.node_receipts.values())
    new_counts=(len(live.rows),len(consumer.bindings),len(consumer.receipts))
    live.retention_passes+=1
    live.retention_reclaimed+=sum(a-b for a,b in zip(old_counts,new_counts))
