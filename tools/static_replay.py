"""Derive project-local static leaves from delivered synthetic raw exchanges.

Only explicitly audited synthetic physical-exchange mappings are supported here. This never reads the
physical/global UWB log, anchor coordinates, or vehicle truth.
"""
import hashlib
import json
import copy
from collections import OrderedDict
from pathlib import Path

from liuo_tunnel_msgs.msg import StaticAnchorRecord, StaticRangeLeaf
from rclpy.qos import QoSProfile, ReliabilityPolicy


class StaticReplayPublisher:
    def __init__(self, node, source, run, registry, config):
        if config.get("source_model") not in {"rail_sim_independent_exchange_v1", "road_sim_global_exchange_v1"}:
            raise ValueError("unsupported static source model")
        files = sorted((source / "export/capture/raw").glob("*.mcap"))
        if len(files) != 1:
            raise ValueError("static source requires one frozen MCAP")
        with files[0].open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if config["source_identity"] != "sha256:" + digest:
            raise ValueError("static source content identity mismatch")
        self.config = config
        self.registry = registry
        self.publisher = node.create_publisher(
            StaticAnchorRecord, config.get("topic", "/liuo/uwb/static_edges"),
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.count = 0
        self.identities = set()
        self.fingerprints = {}
        self.reporter_cache = {}
        self.strategy = config.get('delivery_strategy', 'same_time')
        if self.strategy not in {'same_time', 'cache_sync', 'continuous_report'}:
            raise ValueError('unsupported static delivery strategy')
        self.repeated = 0
        self.opportunities = 0
        self.bytes = 0
        self.cache_limit = 32
        self.run = run
        (run / "metadata/manifests/static_source.json").write_text(json.dumps({
            "source_model": config["source_model"], "raw_sha256": digest,
            "scope": "only AA present in delivered raw batches; no global physical log",
            "identity_basis": "frozen synthetic exchange namespace and original physical source sequence; delivery driver ignored",
            "deployment_epoch": config["deployment_epoch"],
        }, indent=2) + "\n")

    def publish(self, batch):
        for observation in batch.observations:
            if observation.kind != 1:
                continue
            if not observation.source_frame_seq_valid or not observation.pair_measurement_freshness_verified:
                raise ValueError("static source lacks independent physical-exchange evidence")
            a = self.registry[observation.node_a]
            b = self.registry[observation.node_b]
            if a[0] != 1 or b[0] != 1 or observation.reporter_id != observation.node_a:
                raise ValueError("invalid physical endpoints")
            first, second = sorted((a[1], b[1]))
            identity = (f'{self.config["source_identity"]}:{observation.source_session_id}:'
                        f'{observation.source_frame_seq}:{first}:{second}')
            physical_source = a[1]
            if self.config["source_model"] == "road_sim_global_exchange_v1":
                if (observation.time_basis != 2 or not observation.source_hold_age_valid or
                    not observation.source_local_time_valid or not observation.source_system_time_valid or
                    observation.source_session_id != observation.reporter_id + 1 or observation.source_frame_seq == 0):
                    raise ValueError("road source clock/physical provenance mismatch")
                # road_scene uses one global physical index before any delivery.
                # Reporter direction and driver restarts cannot create new leaves.
                identity = (f'{self.config["source_identity"]}:road-exchange:'
                            f'{observation.source_frame_seq}:{first}:{second}')
                physical_source = first  # canonical endpoint of synthetic exchange
            leaf = StaticRangeLeaf()
            leaf.physical_id = identity
            leaf.physical_source = physical_source
            leaf.acquisition_stamp = observation.source_frame_stamp
            leaf.acquisition_stamp_valid = True
            leaf.time_basis = "shared_simulation_clock"
            leaf.range_m = float(observation.distance_m)
            leaf.variance_m2 = self.config["variance_m2"]
            record = StaticAnchorRecord()
            record.header.stamp = batch.header.stamp
            record.network = self.config["network"]
            record.deployment_epoch = self.config["deployment_epoch"]
            record.source_identity = self.config["source_identity"]
            record.record_id = identity
            record.revision = 1
            record.first, record.second = first, second
            record.error_model = "independent-gaussian-v1"
            record.valid = True
            record.covered_blocks = [identity]
            record.leaves = [leaf]
            fingerprint = (first, second, physical_source, leaf.range_m, leaf.variance_m2,
                           leaf.acquisition_stamp.sec, leaf.acquisition_stamp.nanosec, leaf.time_basis)
            if identity in self.fingerprints and self.fingerprints[identity] != fingerprint:
                raise ValueError('same physical identity changed payload')
            is_new = identity not in self.identities
            self.fingerprints[identity] = fingerprint
            self.identities.add(identity)
            if len(self.identities) > self.config.get("max_leaves", 8192):
                raise RuntimeError("static replay physical identity capacity")
            # A successfully delivered raw AA event proves an opportunity for
            # this reporter at this delivery instant. No global LOS/truth log
            # is used, and another reporter's cache is never transmitted here.
            self.opportunities += 1
            reporter = int(observation.reporter_id)
            cached = self.reporter_cache.setdefault(reporter, OrderedDict())
            outgoing = [record] if (self.strategy == 'same_time' or is_new) else []
            if self.strategy == 'continuous_report':
                outgoing.extend(old for key, old in cached.items() if key != identity)
                if not is_new:
                    outgoing.insert(0, record)
            from rclpy.serialization import serialize_message
            for item in outgoing:
                delivered = copy.deepcopy(item)
                delivered.header.stamp = batch.header.stamp
                self.publisher.publish(delivered)
                self.count += 1
                self.bytes += len(serialize_message(delivered))
                self.repeated += int(delivered.record_id != identity or not is_new)
            cached[identity] = record
            cached.move_to_end(identity)
            while len(cached) > self.cache_limit:
                cached.popitem(last=False)

    def close(self):
        (self.run / "export/simulation").mkdir(parents=True, exist_ok=True)
        (self.run / "export/simulation/static_delivery.json").write_text(json.dumps({
            "records": self.count, "unique_physical_ids": sorted(self.identities),
            "strategy": self.strategy,
            "reachability": "same reporter's actual delivered raw AA event only; no interpolation or global LOS data",
            "reporter_cache_limit": self.cache_limit,
            "reporter_cache_peak": max((len(cache) for cache in self.reporter_cache.values()), default=0),
            "delivery_opportunities": self.opportunities,
            "repeated_records": self.repeated,
            "serialized_payload_bytes": self.bytes,
            "physical_sampling": "unchanged frozen raw input; software transmission accounting, not RF/MAC simulation",
        }, indent=2) + "\n")
