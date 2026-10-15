"""Do not subscribe/consume until initial DDS metadata has fully converged.

No incomplete metadata is trusted. This only delays the FIRST binding, with a
deadline; it does not recover or clear a compromised running source binding.
"""
from copy import deepcopy
import time

class AdmissionWindow:
    def __init__(self,specs,start,timeout=20.,stable_s=.5):
        self.specs=specs;self.deadline=start+timeout;self.stable_s=stable_s
        self.candidate=None;self.since=None;self.admitted=False
    def observe(self,snapshot,now):
        if self.admitted:raise ValueError('Startup admission cannot be reused')
        if now>=self.deadline:raise TimeoutError('Managed source metadata did not converge before startup deadline')
        candidate={}
        for role,spec in self.specs.items():
            endpoints=snapshot.get(role,[])
            if len(endpoints)>1:raise ValueError('Multiple publishers before admission: '+role)
            if len(endpoints)!=1:break
            endpoint=endpoints[0];gid=endpoint.get('gid','')
            if endpoint.get('node')!=spec['node'] or endpoint.get('namespace')!=spec['namespace']:break
            if not (isinstance(gid,str) and 32<=len(gid)<=64 and len(gid)%2==0 and
                    set(gid)<=(set('0123456789abcdef')) and any(c!='0' for c in gid)):break
            candidate[role]=gid
        if set(candidate)!=set(self.specs):self.candidate=None;self.since=None;return None
        if candidate!=self.candidate:self.candidate=deepcopy(candidate);self.since=now;return None
        if now-self.since<self.stable_s:return None
        self.admitted=True;return deepcopy(candidate)

def wait_for_sources(node,specs,stop_requested,record):
    import rclpy
    gate=AdmissionWindow(specs,time.monotonic());previous=None
    while True:
        if stop_requested() or not rclpy.ok():raise RuntimeError('Source admission interrupted')
        now=time.monotonic();snapshot={}
        for role,spec in specs.items():
            snapshot[role]=[dict(node=e.node_name,namespace=e.node_namespace,gid=bytes(e.endpoint_gid).hex())
                            for e in node.get_publishers_info_by_topic(spec['topic'])]
        if snapshot!=previous:record(dict(event='discovery',monotonic=now,sources=snapshot));previous=deepcopy(snapshot)
        admitted=gate.observe(snapshot,now)
        if admitted:
            record(dict(event='admitted',monotonic=now,bindings=admitted));return admitted
        rclpy.spin_once(node,timeout_sec=.05)
