"""Bind measured Tag pixels to actual Core receipts; never add a graph edge.

The ROS-facing class owns transport bindings. The detector's original record
and the pure comparison result remain unchanged; transport proof is separate.
This module deliberately has no Info/OdomInfo-based epoch inference.
"""
from collections import OrderedDict
from copy import deepcopy
import hashlib
import json
import math
import time

from core_pixel_join import compare_accepted_image, integer_string, ReceiptJoinError
from tag_pixel_record import validate_pixel_record, digest, PixelRecordError


class ConsumerError(ValueError):
    pass


def require(ok, reason):
    if not ok:
        raise ConsumerError(reason)


def strict_json(raw, maximum=262144):
    require(type(raw) is str and len(raw.encode('utf-8')) <= maximum, 'message size invalid')
    def pairs(items):
        out = {}
        for key, value in items:
            require(key not in out, 'duplicate JSON key')
            out[key] = value
        return out
    def nonfinite(value):
        raise ConsumerError('nonfinite JSON value')
    out = json.loads(raw, object_pairs_hook=pairs, parse_constant=nonfinite)
    require(type(out) is dict, 'expected JSON object')
    return out


def measured_pixels(raw):
    obj = strict_json(raw)
    fields = {'schema_version', 'producer', 'image_stamp_ns', 'camera_frame',
              'observations', 'rejections', 'pose_estimate_provided',
              'graph_update_authorized', 'requires_multiview_validation',
              'corner_measurement_method', 'refinement_details'}
    require(set(obj) == fields, 'unexpected measured-pixel envelope')
    require(type(obj['schema_version']) is int and obj['schema_version'] == 1 and
            obj['producer'] == 'wbrov_apriltag_decoded_pixel_observer', 'wrong pixel producer/version')
    require(obj['pose_estimate_provided'] is False and obj['graph_update_authorized'] is False and
            obj['requires_multiview_validation'] is True, 'pixel evidence boundary invalid')
    require(obj['corner_measurement_method'] == 'verified_image_edges_or_bounded_module_subpix_v1',
            'unsupported corner measurement method')
    stamp = obj['image_stamp_ns']
    require(type(stamp) is int and 0 < stamp <= 2**63-1, 'invalid exact pixel timestamp')
    require(type(obj['camera_frame']) is str and 0 < len(obj['camera_frame']) <= 256, 'invalid camera frame')
    require(type(obj['observations']) is list and len(obj['observations']) <= 128 and
            type(obj['rejections']) is list and type(obj['refinement_details']) is list, 'invalid observation lists')
    seen = set()
    for item in obj['observations']:
        require(type(item) is dict and set(item) == {'tag_id', 'pixel_observation'}, 'invalid pixel item')
        tag = item['tag_id']
        require(type(tag) is int and tag >= 0 and tag not in seen, 'duplicate/invalid Tag ID')
        seen.add(tag)
        validate_pixel_record(item['pixel_observation'], stamp, tag, obj['camera_frame'])
    return obj


class ManagedSources:
    """One endpoint per configured role, bound once for this consumer lifetime.

    Discovery checks describe local observations, not cryptographic peer
    authentication or a distributed lock. Core rechecks its own state at commit.
    """
    def __init__(self, node, specs):
        self.node = node
        self.specs = specs
        self.bound = {}
        self.compromised = False

    @staticmethod
    def gid(raw):
        value = bytes(raw)
        require(16 <= len(value) <= 32 and any(value), 'invalid actual DDS GID')
        return value.hex()

    def check(self, role, incoming=None, claimed=None):
        require(not self.compromised, 'source binding compromised; restart managed consumer')
        spec = self.specs[role]
        endpoints = self.node.get_publishers_info_by_topic(spec['topic'])
        if len(endpoints) != 1:
            if len(endpoints) > 1:
                self.compromised = True
            raise ConsumerError(role + ': expected exactly one current publisher')
        endpoint = endpoints[0]
        gid = self.gid(endpoint.endpoint_gid)
        if (endpoint.node_name != spec['node'] or endpoint.node_namespace != spec['namespace'] or
                (role in self.bound and self.bound[role] != gid)):
            self.compromised = True
            raise ConsumerError(role + ': publisher replaced or not the managed node')
        require(incoming is None or self.gid(incoming.publisher_gid) == gid, role + ': message GID mismatch')
        require(claimed is None or claimed == gid, role + ': receipt sensor GID mismatch')
        self.bound.setdefault(role, gid)
        return gid

    def all_current(self):
        for role in self.specs:
            self.check(role)
        return dict(self.bound)


def make_consumer(node, config, emit):
    """Attach actual ROS subscriptions to a supplied managed node.

    Config is supplied by the launcher, never copied from an incoming message.
    emit writes/forwards a complete evidence event. No graph API is called here.
    """
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

    class Consumer:
        def __init__(self):
            self.sources = ManagedSources(node, config['sources'])
            self.session = config['session_token_sha256']
            self.database = config['database_sha256']
            self.extrinsic = config['body_from_camera']
            require(type(self.session) is str and len(self.session) == 64 and self.session != '0'*64 and
                    all(c in '0123456789abcdef' for c in self.session), 'invalid configured session')
            require(type(self.database) is str and len(self.database) == 64 and
                    all(c in '0123456789abcdef' for c in self.database), 'invalid configured database')
            require(type(self.extrinsic) is list and len(self.extrinsic) == 12 and
                    all(type(v) in (int, float) and math.isfinite(v) for v in self.extrinsic), 'invalid configured extrinsic')
            self.max_age = float(config.get('pending_max_age_s', 8))
            require(math.isfinite(self.max_age) and 0 < self.max_age <= 30, 'invalid pending age')
            self.pending = OrderedDict()
            self.receipts = OrderedDict()
            self.node_receipts = {}
            self.bindings = {}
            self.counts = {'bound': 0, 'rejected': 0, 'duplicate': 0, 'expired': 0}
            self.stopped = False
            self.reported_compromise = False
            volatile = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE)
            retained = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscriptions = [
                node.create_subscription(String, config['sources']['core']['topic'], self.on_receipt, retained),
                node.create_subscription(String, config['sources']['pixels']['topic'], self.on_pixels, volatile)]
            for subscription in self.subscriptions:
                subscription._wbrov_message_info_required = True
            self.timer = node.create_timer(.1, self.expire)

        def event(self, kind, **data):
            emit(dict(schema_version=1, producer='wbrov_authenticated_core_pixel_consumer',
                      event=kind, receipt_monotonic_ns=str(time.monotonic_ns()),
                      graph_update_authorized=False, **data))

        def rejected(self, reason):
            self.counts['rejected'] += 1
            if self.sources.compromised:
                self.pending.clear()
                if not self.reported_compromise:
                    self.reported_compromise = True
                    self.event('sources_invalidated', reason=str(reason), previous_bindings=len(self.bindings),
                               previously_emitted_proof_is_current=False)
            self.event('rejected', reason=str(reason))

        def check_receipt_context(self, receipt):
            require(type(receipt) is dict, 'invalid receipt')
            require(receipt.get('session_token_sha256') == self.session and
                    receipt.get('database_sha256') == self.database, 'receipt session/database mismatch')
            require(receipt.get('source_scope') == 'managed_ros_rgbd', 'unsupported receipt scope')
            require(receipt.get('camera_frame') == config['camera_frame'], 'wrong managed camera frame')
            require(type(receipt.get('node_id')) is int and 0 < receipt['node_id'] <= 2147483647, 'invalid Core node ID')
            integer_string(receipt.get('reset_epoch'), 2**64-1)
            ns = integer_string(receipt.get('header_stamp_ns'), 2**63-1, True)
            self.sources.check('rgb', claimed=receipt.get('rgb_publisher_gid'))
            self.sources.check('camera_info', claimed=receipt.get('camera_info_publisher_gid'))
            return ns

        def on_receipt(self, message, info):
            try:
                require(not self.stopped, 'consumer is stopped')
                source = self.sources.check('core', incoming=info)
                receipt = strict_json(message.data, 16384)
                ns = self.check_receipt_context(receipt)
                # Never turn an inconsistent node/timestamp into a new identity.
                key = receipt['node_id']
                require(key not in self.node_receipts or self.node_receipts[key] == receipt, 'conflicting Core node receipt')
                require(ns not in self.receipts or self.receipts[ns]['receipt'] == receipt, 'conflicting Core timestamp receipt')
                if ns in self.receipts:
                    self.counts['duplicate'] += 1
                    self.event('duplicate_receipt', node_id=key)
                    return
                require(len(self.receipts) < 4096, 'receipt capacity reached')
                self.node_receipts[key] = deepcopy(receipt)
                self.receipts[ns] = dict(receipt=deepcopy(receipt), raw_sha256=hashlib.sha256(message.data.encode()).hexdigest(),
                                         source_gid=source, arrival=time.monotonic())
                self.match(ns)
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
                self.rejected(exc)

        def on_pixels(self, message, info):
            try:
                require(not self.stopped, 'consumer is stopped')
                source = self.sources.check('pixels', incoming=info)
                pixels = measured_pixels(message.data)
                ns = pixels['image_stamp_ns']
                require(pixels['camera_frame'] == config['camera_frame'], 'wrong managed pixel frame')
                if not pixels['observations']:
                    self.event('no_measured_tags', image_stamp_ns=str(ns))
                    return
                entry = dict(payload=deepcopy(pixels), source_gid=source, arrival=time.monotonic(),
                             raw_sha256=hashlib.sha256(message.data.encode()).hexdigest())
                if ns in self.pending:
                    require(self.pending[ns]['payload'] == pixels, 'conflicting pending pixel message')
                    self.counts['duplicate'] += 1
                    return
                require(len(self.pending) < 256, 'pending pixel capacity reached')
                self.pending[ns] = entry
                self.match(ns)
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
                self.rejected(exc)

        def match(self, ns):
            if ns not in self.receipts or ns not in self.pending:
                return
            pending = self.pending.pop(ns)
            require(time.monotonic()-pending['arrival'] <= self.max_age, 'pending pixels expired')
            accepted = self.receipts[ns]
            receipt = accepted['receipt']
            self.check_receipt_context(receipt)
            current = self.sources.all_current()
            require(current['core'] == accepted['source_gid'] and current['pixels'] == pending['source_gid'],
                    'queued source changed before binding')
            events = []
            for item in pending['payload']['observations']:
                record = item['pixel_observation']
                comparison = compare_accepted_image(receipt, record,
                    expected_session_sha256=self.session, expected_database_sha256=self.database,
                    expected_body_from_camera=self.extrinsic)
                key = (receipt['node_id'], item['tag_id'])
                if key in self.bindings:
                    require(self.bindings[key]['pixel_observation'] == record and
                            self.bindings[key]['accepted_image_receipt'] == receipt, 'conflicting bound pixel observation')
                    self.counts['duplicate'] += 1
                    self.event('duplicate_binding', node_id=key[0], tag_id=key[1])
                    continue
                require(len(self.bindings)+len(events) < 4096, 'binding capacity reached')
                result = dict(keyframe_id=key[0], tag_id=key[1], reset_epoch=receipt['reset_epoch'],
                    image_stamp_ns=str(ns), accepted_image_receipt=deepcopy(receipt), pixel_observation=deepcopy(record),
                    identity_comparison=comparison,
                    transport_proof=dict(core_message_gid=accepted['source_gid'], pixel_message_gid=pending['source_gid'],
                        core_message_sha256=accepted['raw_sha256'], pixel_message_sha256=pending['raw_sha256'],
                        current_endpoints=current, session_token_sha256=self.session, database_sha256=self.database,
                        core_receipt_transport_verified=True, pixel_transport_verified=True,
                        provenance_scope='current_consumer_session', discovery_is_point_in_time=True),
                    requires_current_core_graph_validation=True, requires_multiview_validation=True,
                    usable_as_absolute_anchor=False)
                events.append((key, result))
            for key, result in events:
                self.bindings[key] = deepcopy(result)
                self.counts['bound'] += 1
                self.event('bound', **result)

        def receipts_for_endpoints(self, ids, tag_id):
            """Forward owned receipts; this does not grant geometric acceptance."""
            require(not self.stopped and len(ids) == 6 and len(set(ids)) == 6, 'six unique endpoints required')
            require(type(tag_id) is int and all(type(i) is int and i > 0 for i in ids), 'invalid endpoint/Tag IDs')
            current = self.sources.all_current()
            out = []
            for node_id in ids:
                require((node_id, tag_id) in self.bindings, 'endpoint lacks measured pixel binding')
                record = self.bindings[(node_id, tag_id)]
                require(record['transport_proof']['current_endpoints'] == current, 'bound sources changed before receipt forwarding')
                self.check_receipt_context(record['accepted_image_receipt'])
                out.append(deepcopy(record['accepted_image_receipt']))
            return out

        def corner_observations_for_endpoints(self, ids, tag_id):
            """Forward measured ordered pixels bound to the same owned receipts."""
            require(not self.stopped and len(ids) == 6 and len(set(ids)) == 6,
                    'six unique endpoints required')
            current = self.sources.all_current()
            out = []
            for node_id in ids:
                require((node_id, tag_id) in self.bindings,
                        'endpoint lacks measured pixel binding')
                binding = self.bindings[(node_id, tag_id)]
                require(binding['transport_proof']['current_endpoints'] == current,
                        'bound sources changed before corner forwarding')
                self.check_receipt_context(binding['accepted_image_receipt'])
                pixel = binding['pixel_observation']
                receipt = binding['accepted_image_receipt']
                require(pixel['image']['sha256'] == receipt['rgb_sha256'],
                        'measured corners do not match the accepted image digest')
                out.append({
                    'signature_id': node_id,
                    'tag_id': tag_id,
                    'corner_order': pixel['corner_order'],
                    'pixel_record_sha256': pixel['sha256'],
                    'image_sha256': pixel['image']['sha256'],
                    'corners_xy': [float(value) for point in pixel['corners_xy'] for value in point],
                })
            return out

        def expire(self):
            now = time.monotonic()
            for ns in list(self.pending):
                if now-self.pending[ns]['arrival'] > self.max_age:
                    self.pending.pop(ns)
                    self.counts['expired'] += 1
                    self.event('expired_pixels', image_stamp_ns=str(ns))

        def close(self):
            self.stopped = True
            self.pending.clear()
            node.destroy_timer(self.timer)
            for subscription in self.subscriptions:
                node.destroy_subscription(subscription)
            self.event('closed', counts=dict(self.counts), source_compromised=self.sources.compromised)

    return Consumer()
