# UWB observation input contract

Implementation in progress; acceptance status is recorded in
[unification results](../uwb_observation_unification_results.md).

The only production UWB subscription is `liuo_uwb_msgs/msg/UwbObservationBatch`
at `/liuo/uwb/observations`. The interface comes from the adjacent read-only
`nlink_parser_uwb/packages/liuo_uwb_msgs` package, built into the explicit data-disk
message underlay. No bridge to UwbRange is used. UwbRange remains available for
historical decoding only; new replay rejects legacy bags and requires the
historical LIUO revision to replay them.

## Configuration and meanings

`uwb.input` declares one network, receiver, explicit logical-to-typed-internal
registry and an optional model. The simulation registry maps N0 to tag 0 and
N1…N7 to anchors 0…6. Coordinates, map indices and installation order are not
inferred from logical IDs. The simulator reads the same registry in reverse.
Existing bundled configurations are explicitly simulation configurations. Their
noise is a synthetic assumption (TA/AA sigma 0.10 m), not measured calibration.
A hardware configuration must supply its own network, registry and evidence;
without a model observations are recorded and rejected for fusion.

A model names its source, clock domain, timing, noise and independence evidence,
TA/AA sigma in meters, and maximum accepted time uncertainty in seconds. Invalid
or incomplete model parameters fail startup. Unknown/receive-only time, illegal
source timestamps, nonfinite uncertainty and unverified freshness are rejected.
Neither batch Header, received_stamp nor callback time substitutes for source time.
RSSI is not a LOS classifier. Unknown NaN/false fields remain unchanged.

The batch Header is publication time. Individual source_frame_stamp is used for
association. Driver received_stamp remains in the original serialized record;
callback time is independently recorded. `uwb_batches.csv` preserves every batch
as CDR hex plus callback time, batch identity, item count and batch-level reason.
`uwb.csv` records per-item decisions, mapped endpoints, network and driver session.
Callbacks validate and enqueue; the existing estimator update owns graph changes.

## Identity, resource bounds and AA

Identity includes configured network, full uint64 driver/source sessions,
reporter, valid uint32 source sequence, kind and endpoints. Batch sequence,
transmission cycle and shards do not refresh identity. Same-frame different peers
are accepted immediately. Local batches are not restricted to five entries;
1024 is the explicit input batch cap. Invalid entries do not reject their peers.

Each registered reporter retains a 128-sequence sliding window, allowing 127
frames of reordering, with modular uint32 progression and maximum forward jump
1,000,000. Window retirement is permanent through its high-water mark. Sessions
are full-width values; 64 retired source sessions per reporter and 64 retired
driver datasets are retained. Exhaustion rejects further data rather than
forgetting evidence and resurrecting old reports. ROS reset retires the dataset.
The input registry bounds reporters and endpoints; the pending range queue is
bounded independently. No arrival counter manufactures measurement identities.

Freshness is required but not inferred from source sequence. Static AA locks to
the first available directed reporter for each unordered edge in a dataset;
reverse reports remain logged but add no information. This accepts larger-ID-only
coverage. Accepted independent samples use the existing exact Gaussian summary
(sum precision, weighted mean and scatter); retransmits and alternate-direction
reports never update it. Unknown-time AA is rejected. AA factors contain fixed
anchors only. `use_aa=false` and record-only remain explicit modes.

## Wire precision and tools

The simulator keeps the original exchange schedule and independent noise streams.
Each actual sample creates one batch with exact simulated source time, zero
uncertainty and verified freshness. RSSI is NaN/false; hold age is invalid/65535.
Float32 conversion occurs after physical double-precision distance generation;
there is no additional millimeter quantization.

Semantic hash schema `liuo_uwb_observation_batch_v1` includes all declared fields
recursively, with stable strings for NaN and infinities. Batch, observation,
unique-report and graph counts have distinct meanings. Replay keeps original
source/received/publication timestamps. Delays and duplicates preserve identity;
anchor outages filter individual entries. NLOS stress changes only float32
`distance_m`; original distance and synthetic bias are recorded in delivery.csv.
It cannot set nonexistent message variance or propagation flags.

Default QoS is best effort, volatile, keep last 100. The production node exposes
`uwb_topic` and `uwb_qos_depth`; ROS remapping also applies. Tool-wide configurable
topic/depth propagation and complete lifecycle/load integration remain subject to
acceptance tests, not certified by this document.
