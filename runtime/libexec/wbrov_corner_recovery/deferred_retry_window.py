"""Bounded proposal scheduling, never permission to merge a graph.

Core independently rechecks all original evidence, depth, gravity and shadows.
Only authenticated accepted-image receipts feed this source-clock window.
"""
class DeferredRetryWindow:
    def __init__(self):
        self.epoch=None;self.nodes=set();self.first=None;self.latest=None
        self.attempts=0;self.last_attempt=None;self.last_check=None
    def observe(self,epoch,node_id,stamp):
        if self.epoch is not None and epoch<self.epoch:return
        if epoch!=self.epoch:
            self.epoch=epoch;self.nodes=set();self.first=None;self.latest=None
            self.attempts=0;self.last_attempt=None;self.last_check=None
        if type(stamp) is not int or stamp<=0 or type(node_id) is not int or node_id<=0:
            raise ValueError('Invalid authenticated receipt identity')
        if len(self.nodes)<10:self.nodes.add(node_id)
        self.first=stamp if self.first is None else min(self.first,stamp)
        self.latest=stamp if self.latest is None else max(self.latest,stamp)
    def eligible(self,epoch,revision,rejected_revision):
        return bool(epoch==self.epoch and len(self.nodes)>=10 and self.latest-self.first>=15_000_000_000 and
            revision>rejected_revision and self.attempts<2 and
            (self.last_attempt is None or self.latest-self.last_attempt>=10_000_000_000) and
            (self.last_check is None or self.latest>self.last_check))
    def checked(self):self.last_check=self.latest
    def submitted(self):
        if self.attempts>=2:raise ValueError('Retry budget exhausted')
        self.attempts+=1;self.last_attempt=self.latest
