"""Validate owned Core terminal status without inventing parsed evidence."""
def require(condition,message):
    if not condition:raise ValueError(message)

def validate_terminal(s,p,config):
    fields={'schema_version','producer','result','reason','transaction_token','tag_id','source_reset_epoch',
        'observer_graph_revision','core_reset_epoch','core_graph_revision','database_sha256','session_token_sha256','evidence_pairs'}
    require(type(s) is dict and set(s)==fields,'Core status fields')
    require(s['producer']=='wbrov_rtabmap_safe_relocalization_core' and type(s['schema_version']) is int and s['schema_version']==1,'Core status identity')
    require(type(s['reason']) is str and s['reason'],'terminal reason')
    require(s['transaction_token']==p['transaction_token'] and bool(s['transaction_token']),'terminal token')
    require(s['session_token_sha256']==config['session_token_sha256'] and s['database_sha256']==p['anchor_database_sha256'],'terminal session')
    require(type(s['tag_id']) is int and s['tag_id']==p['tag_id'],'terminal Tag')
    for key in ('source_reset_epoch','observer_graph_revision','core_reset_epoch','core_graph_revision'):
        require(type(s[key]) is int and s[key]>=0,'terminal integer identity')
    require(s['source_reset_epoch']==p['source_reset_epoch'] and s['observer_graph_revision']==p['observer_graph_revision'],'terminal epoch/revision')
    require(s['result'] in ('committed','rejected'),'terminal state')
    pairs=s['evidence_pairs'];expected=[[e['from_id'],e['to_id']] for e in p['evidence']]
    require(type(pairs) is list and len(pairs)<=3 and all(type(v) is list and len(v)==2 and all(type(i) is int and i>0 for i in v) for v in pairs),'terminal link types')
    # Parser failures report only links appended before the failing item.
    # They are a rejection, and never evidence that omitted links were checked.
    require(pairs==expected[:len(pairs)],'terminal parsed link prefix')
    if s['result']=='committed':require(pairs==expected,'committed full links')
    require(s['core_reset_epoch']==p['source_reset_epoch'],'Core epoch changed during synchronous replay proposal')
    require(s['core_graph_revision']==p['observer_graph_revision']+(s['result']=='committed'),'terminal graph revision')


def validate_observing(s,p,config):
    """Authenticate a prepared, nonterminal candidate without relaxing terminal validation."""
    fields={'schema_version','producer','result','reason','transaction_token','tag_id','source_reset_epoch',
        'observer_graph_revision','core_reset_epoch','core_graph_revision','database_sha256','session_token_sha256','evidence_pairs'}
    require(type(s) is dict and set(s)==fields,'Core observing status fields')
    require(s['producer']=='wbrov_rtabmap_safe_relocalization_core' and type(s['schema_version']) is int and s['schema_version']==1,'Core observing status identity')
    require(type(s['reason']) is str and s['reason'],'observing reason')
    require(s['result']=='observing','observing state')
    require(s['transaction_token']==p['transaction_token'] and bool(s['transaction_token']),'observing token')
    require(s['session_token_sha256']==config['session_token_sha256'] and s['database_sha256']==p['anchor_database_sha256'],'observing session')
    require(type(s['tag_id']) is int and s['tag_id']==p['tag_id'],'observing Tag')
    for key in ('source_reset_epoch','observer_graph_revision','core_reset_epoch','core_graph_revision'):
        require(type(s[key]) is int and s[key]>=0,'observing integer identity')
    require(s['source_reset_epoch']==p['source_reset_epoch'] and s['observer_graph_revision']==p['observer_graph_revision'],'observing epoch/revision')
    pairs=s['evidence_pairs'];expected=[[e['from_id'],e['to_id']] for e in p['evidence']]
    require(type(pairs) is list and len(pairs)==3 and all(type(v) is list and len(v)==2 and all(type(i) is int and i>0 for i in v) for v in pairs),'observing full link types')
    require(pairs==expected,'observing full links')
    require(s['core_reset_epoch']==p['source_reset_epoch'],'Core epoch changed during observing proposal')
    require(s['core_graph_revision']==p['observer_graph_revision'],'observing graph must remain uncommitted')
