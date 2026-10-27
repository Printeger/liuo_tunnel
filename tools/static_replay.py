"""Derive project-local static leaves from delivered synthetic raw exchanges.

Only the audited rail simulator mapping is supported here. This never reads the
physical/global UWB log, anchor coordinates, or vehicle truth.
"""
import hashlib
import json
from pathlib import Path

from liuo_tunnel_msgs.msg import StaticAnchorRecord, StaticRangeLeaf
from rclpy.qos import QoSProfile, ReliabilityPolicy


class StaticReplayPublisher:
    def __init__(self, node, source, run, registry, config):
        if config.get("source_model") != "rail_sim_independent_exchange_v1":
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
            leaf = StaticRangeLeaf()
            leaf.physical_id = identity
            leaf.physical_source = a[1]
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
            self.publisher.publish(record)
            self.count += 1
            self.identities.add(identity)
            if len(self.identities) > self.config.get("max_leaves", 8192):
                raise RuntimeError("static replay physical identity capacity")

    def close(self):
        (self.run / "export/simulation/static_delivery.json").write_text(json.dumps({
            "records": self.count, "unique_physical_ids": sorted(self.identities),
            "strategy": "same-time derivation from delivered raw AA only",
        }, indent=2) + "\n")
