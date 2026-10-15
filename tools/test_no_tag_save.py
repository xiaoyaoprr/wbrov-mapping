from pathlib import Path
import argparse,json,subprocess,tempfile,copy
p=argparse.ArgumentParser();p.add_argument('--mapping',type=Path,required=True);a=p.parse_args()
text=a.mapping.read_text(encoding='utf-8');start=text.index('no_exact_tag_observations() {');stop=text.index('build_apriltag_multiview_sidecar() {',start);function=text[start:stop]
with tempfile.TemporaryDirectory(prefix='wbrov-no-tag-') as tmp:
 r=Path(tmp);state_path=r/'apriltag_keyframe_binding_state.json';rows_path=r/'apriltag_keyframe_bindings.jsonl'
 state={'schema_version':1,'pid':123,'observer_only':True,'graph_edge_added':False,'output':str(rows_path),'accepted_bindings':0,'pending_info_stamps':0,'pending_tag_stamps':0,'rejected_events':1}
 event={'accepted':False,'reason':'info_without_exact_tag','keyframe_id':1,'image_stamp_ns':42,'reset_epoch':0}
 def check(st,ev,expected,continuation='false',missing=False):
  state_path.write_text(json.dumps(st));rows_path.write_text(json.dumps(ev)+'\n')
  if missing:rows_path.unlink()
  script='run_dir="$1"\npython3_runtime=/usr/bin/python3\napriltag_keyframe_binder_pid=123\ncontinuation_enabled="$2"\n'+function+'\nno_exact_tag_observations\n'
  q=subprocess.run(['/bin/bash','-c',script,'test',str(r),continuation],capture_output=True,text=True,timeout=5)
  assert (q.returncode==0)==expected,(q.returncode,q.stderr,st,ev)
 check(state,event,True)
 check(state,event,False,continuation='true')
 check(state,event,False,missing=True)
 for key,value in [('pid',99),('accepted_bindings',1),('pending_tag_stamps',1),('observer_only',False),('rejected_events',2)]:
  st=copy.deepcopy(state);st[key]=value;check(st,event,False)
 for patch in ({'accepted':True},{'reason':'tag_without_exact_info'},{'tag_id':3},{'keyframe_id':0}):
  ev=dict(event,**patch);check(state,ev,False)
 # The actual on_exit branch accepts only checked new-map absence; a missing
 # continuation receipt still rejects every non-absence path.
 branch=text[text.index('    if ((cleanup_status == 0 && lifecycle_failed == 0))'):text.index('    # A second launcher must remain excluded until immediately after this')]
 for absent,continued,expected in [('true','false',True),('false','false',False),('true','true',False)]:
  setup='cleanup_status=0; lifecycle_failed=0; critical_failure_reason=""; requested_status=0; status=0; succeeded=false; cleanup_nodes_stopped=true; cleanup_export_succeeded=true; cleanup_continuation_contract_succeeded=false; run_dir="$1"; cleanup_no_tag_available="$2"; continuation_enabled="$3"\ncontinuation_contract_tool() { return 1; }\n'
  q=subprocess.run(['/bin/bash','-c',setup+branch+'\n[[ "$succeeded" == true && "$status" == 0 ]]','test',str(r),absent,continued],capture_output=True,text=True,timeout=5)
  assert (q.returncode==0)==expected,(q.returncode,q.stderr)
print(json.dumps({'status':'passed','cases':15,'actual_launcher_code':True,'hardware_started':False}))
