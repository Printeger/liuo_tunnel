# LIUO run artifact contract

Normative. Adapted from IAP `docs/spec/run_artifact_contract.md` at `fbccae16802402d7300bb817d2627f86e6a495df`. Schema: `liuo_tunnel_run_artifact_v1`.

One owner allocates and finalizes a run. `LIUO_RUN_ROOT` is the public absolute root; source runs default to this project's `log/`, installed standalone applications to `${XDG_STATE_HOME:-~/.local/state}/liuo_tunnel/log`. Internal `LIUO_RUN_DIR` identifies the exact existing run adopted by every child. Children never allocate a nested run, change `latest`, or finalize the primary manifest.

IDs are UTC `YYYYMMDDTHHMMSSZ_mmm`, with optional collision suffix `_NN`. Allocation uses atomic mkdir; latest updates and retention use root locks. `latest` is a relative, atomically replaced symlink. A regular file or directory named latest is preserved and causes failure. Non-run tooling symlinks do not outrank actual run IDs.

Only these first-level directories are allowed inside a run:

- `runtime/`: `liuo_<module>.log`, child console logs, `ros/`. `ROS_LOG_DIR` must point to `runtime/ros`.
- `profiling/`: renderer, preprocessing, optimization, queue, latency and resource diagnostics.
- `export/odometry/`: online trajectory, raw UWB receipt decisions, graph counts and LiDAR information spectra.
- `export/simulation/`: truth, generated ranges, scheduling, visibility and noise evidence.
- `export/capture/`: raw IMU CSV and explicitly requested raw pointcloud/bag capture.
- `export/analysis/`: offline reports, metrics and derived diagnostics; never overwrite online module outputs.
- `metadata/`: the primary manifest, `config/`, `processes/` and `manifests/`.

`metadata/run_manifest.json` is the sole run-level authority. The owner writes it atomically via temporary sibling and rename. It binds schema, run ID/root/dir, entrypoint, scenario, modules, actual UTC start/end, lifecycle, source commit and dirty state, build/host identity, config references, subordinate manifests, run/retention class and external exports. Simulation timestamps are a separate clock domain. Input identity binds the seed, scene, effective sensor parameters, scan model, installed executables, dependency paths and hashes. Child process records include PID and adopted directory.

Lifecycle is `active`, `completed`, `failed` or `interrupted`. Acceptance is separately `not_evaluated`, `pass`, `fail` or `incomplete`; completed never implies pass. An unexpected required process exit creates retained `input_failure.json`, which takes priority over a later apparently successful shutdown. Stop only owned child processes. Never use broad pkill or delete a failed run to make a rerun appear successful.

The path resolver accepts a category and relative leaf. Reject absolute leaves, `..`, symlinks escaping the category and symlinked categories escaping the run. Do not infer output roots from cwd or use legacy CSV paths as output directories. Only explicit exports may leave a run; the owner must record requester, absolute path, kind and time. No such export is part of the default phase-one flow.

Ordinary retention applies only to recognized, direct, nonsymlink development runs with valid identity, ordinary retention class, terminal lifecycle and end time. Keep the newest three and all runs ended within seven days. Active/locked, protected/formal, malformed, symlinked or externally owned directories are never ordinarily collected. External roots require explicit `LIUO_RETENTION_ENABLED=1`. Revalidate under the retention lock, rename to root-local quarantine, then remove only that candidate. A protected run may pin source run IDs. The implementation and tests are migrated from IAP.

Phase-one UWB is `RECORD_ONLY`: no associated graph keys, residual, innovation or robust weight is invented. Raw range values above 800 m remain valid when the pre-noise geometric range was within the limit. Publish per-link effective timestamps, session/sequence identities and actual scheduling/visibility counts. Truth may be consumed only by simulation and independent evaluation. Snapshot point times are zero; rolling/deskew remains explicitly unverified.

Build/test tooling uses ignored `log/build`, `log/test` and `log/tooling` directories, which are not runtime runs and are ineligible for retention. Test-created temporary directories are automatically cleaned. Frozen failure evidence remains unchanged; later reports must explicitly supersede an incorrect historical interpretation instead of rewriting it.

Phase-two frozen input: `export/capture/raw` contains the original MCAP messages,
including measurement identities and clock resets, with a monotonic playback
schedule. `metadata/manifests/captured_inputs.json` binds file SHA256;
`frozen_input.json` binds replay source, capture hashes, received raw stream
hashes and counts. Protected replay manifests retain their source run IDs.
A failed/incomplete capture cannot be promoted to a source. The owner starts
either simulation or replay, never both. `export/simulation/wall_clock.csv`
records session ordinal, effective stamp and scheduled/published monotonic ns;
online trajectory `publish_wall_ns` measures publication against that deadline.
Simulation-time latency, wall end-to-end latency and processing duration are
reported separately. Snapshot scope remains explicit.

### Declared UWB transport replay

Optional `metadata/config/delivery.json` is snapshotted and hashed. The bounded
replay scheduler retains original bytes and effective measurement time; its
separate monotonic delivery deadline may reorder messages. Every omission is an
explicit outage action, every duplicate preserves its physical identity.
`export/simulation/delivery.csv` records identity, effective time, source/delivery
elapsed time and action. Publisher runtime records actual delivered hashes and
expected duplicates; observer hashes must match these, while unchanged topics
must still match the frozen source. Such a run reports `delivered_input_verified`
and does not claim unmodified `frozen_input_verified` for the altered UWB stream.
Estimator `uwb.csv` records decision wall time and rejection reasons; shutdown
pending samples are accounted explicitly. Trajectory `state_reason` records
absence even when the public state was already DEGRADED. The delivery audit
checks unique ingestion, expired/duplicate rejection, output continuity and
first fresh TA recovery; navigation accuracy and timing remain independent gates.

Simulation map fixtures may declare `simulation_anchor_layout`. The owner rejects
fixtures whose layout differs from the selected live scene or frozen capture.
This metadata check does not derive estimator coordinates from IDs or require
external field-survey maps to reveal simulation ground truth.

### Body twist output

`trajectory.csv` appends `wx,wy,wz,twist_interval_s`. Linear velocity is the current
optimized velocity expressed in the child frame. Angular velocity is the SO(3)
log increment over the two adjacent optimized poses divided by their time interval,
expressed in the current child frame; it is explicitly an interval estimate.
`covariance.csv` appends all `twist_i_j` entries in ROS [linear,angular] order,
propagated from the joint previous-pose/current-pose/world-velocity covariance.
No independence approximation removes cross terms. Before a second pose exists,
INITIALIZING/WAIT_SECOND_POSE_FOR_TWIST is published and Odometry is withheld.
An independent `/liuo/odom` subscriber checks finite, symmetric positive covariance
and records wire twist/covariance in `export/capture/odometry.csv`. Evaluation
compares these with the estimator exports and requires equal publication counts.

### Full-route resource evidence

Live `--return-trip` is snapshotted as `scenario.return_trip` and
`motion=outbound_and_reverse_return`; frozen replay preserves its source scenario.
`profiling/resources.csv` samples at 1 Hz: session/stamp, estimator-process RSS
in KiB, whole CUDA device used bytes, retained cloud frames/keyframes and current
IMU/cloud/UWB queues. Device usage is explicitly not process-attributed. Resource
sampling time is included in the processing latency measurement.

### Static AA sufficient statistics

In surveyed mode, independent Gaussian ranges for each static anchor pair share
one distance prediction. Keep weight W=sum(1/variance), weighted mean, count and
weighted scatter S. One replaceable factor represents exactly
`W*(mean-distance(Ai,Aj))^2 + S`, including the constant scatter in its error.
Each accepted physical sample remains in `uwb.csv` with its original identity,
variance and raw pre-optimization innovation. No map prior is repeated. This
aggregation is not valid for moving TA predictions or per-sample robust losses;
neither is aggregated. Oracle AA remains consistency-only.

`graph.csv` appends `aa_summary_factors` and `aa_accumulated_measurements`.
Each update replaces affected pair factors using returned smoother factor indices;
the number of AA factors is bounded by the map's unordered pair count. Clock reset
clears both summaries and indices. Evaluation checks pair bounds, total factor
bounds, and (without a clock reset) accumulated count against accepted raw IDs.

### Independent long-route and paired audits

`export/analysis/long_run_audit.json` is generated independently after a run. It
checks declared-initialization availability, route/stops/return coverage, raw and
wire contracts, numerical per-run failures, structural bounds and continuous
1 Hz resource sampling. Its scope explicitly excludes complete stage acceptance.
The known pending-campaign marker is recorded separately, not confused with a
physical per-run failure or removed from the primary report.
`matched_comparison.json` separately records raw-input equality, LIO setting
identity, binary identity and fused-UWB setting identity. Strict comparison rejects
configuration changes; identical random seeds or raw data alone are insufficient
for a mode-only comparison. Survey fixture seed and map ID identify independent
survey realizations while preserving the provided joint covariance.

Odometry state transitions are retained in `export/odometry/status.csv`, including
session, measurement stamp, operational state/reason, cumulative rejected count
and last rejection. Diagnostics also expose the rejection count/reason when fresh
accepted TA still permits TRACKING. Missing accepted TA reports
`UWB_ABSENT_LIO_CONTINUES`; recent rejected input instead reports
`UWB_REJECTED_LIO_CONTINUES:<reason>`. Neither claims a corrected position or resets
the graph. Weak/nonfinite/unbounded posterior diagnostics have separate reasons;
TRACKING denotes active fusion, not calibrated accuracy or guaranteed truth error.

Fatal exceptions additionally retain `runtime/odometry_failure.json` with the
original detail and a categorized startup/map/extrinsic, preprocessing, IMU,
initialization, optimization, covariance or output failure. A constructed node
emits LOST before terminating; a constructor failure has no usable publisher and
is evidenced by the failure file and process exit. The owner still marks the run
failed. These files do not replace raw per-measurement decisions or the manifest.

After an accepted-TA measurement gap exceeding 0.3 s, operational recovery
requires one full lag with renewed input before TRACKING. Status CSV and
diagnostics expose `uwb_recovery_until`; `UWB_RECOVERING_WINDOW` is DEGRADED.
Missing/rejected input and invalid posterior still take precedence. No state or
covariance is changed by this reporting rule.

`export/analysis/pressure_audit.json` preserves the original report failures and
separates nominal accuracy comparisons from Handbook 8.1 pressure behavior.
It reports outage/recovery errors and timing, checks delivered-input identity and
recovery states, and does not claim nominal or complete-stage acceptance.

Delivery profiles may additionally declare `anchor_outages` with explicit
`anchor_id`, `begin_s`, `end_s` inside the profile window. Both TA and AA links
involving that anchor are withheld in the half-open interval; other endpoints,
measurement payloads, effective stamps and identities remain unchanged.
`delivery.csv` appends `a_type,a,b_type,b` and uses `anchor_outage_drop`. The
auditor checks all original events, including outside the declared interval,
for missed selections and unintended drops. This is a physical link-loss
pressure condition, not a changed nominal range/noise model.

Campaign progress writes are atomic. Explicitly reused completed triplets retain
the original run IDs; they must pass independent audits and match the current
estimator binary/settings and declared independent survey draw. Failed or
partial triplets are rejected. Disk preflight and 5 s reserve monitoring preserve
`INSUFFICIENT_EVIDENCE_SPACE` / `DISK_RESERVE_BELOW_2_GIB` reasons; the campaign
signals only its own Popen owner and waits for owner finalization. No run cleanup
is implied. The original failed campaign remains separate evidence.

Single-anchor fault audits also require other-anchor TA ingestion during the
fault and restored TA ingestion from the failed anchor afterward. Restoration
latency is reported, without imposing an invented per-link latency gate.
Pressure navigation reports preserve outage and first-10-second recovery errors
and states separately; a single-anchor loss does not imply all UWB is absent.

`nlos_bias` is an explicit measurement-pressure transform applied after frozen
transport scheduling, with a declared anchor, interval and independent seed.
Each link has a stationary lognormal AR(1) series, held over `step_s`; the latent
Gaussian correlation is exp(-step_s/correlation_s), and subtracting half the
latent variance preserves the declared ensemble mean at each grid sample.
Per-link series are capped at 10,000 values and aggregate storage at 65,536.
Query order, duplicates and delivery delays do not change a physical bias.
This is a synthetic stress model, not an empirically calibrated NLOS likelihood.

Replay retains base/post-transform range, base/post variance, base/post
propagation status and injected bias in `delivery.csv`. Only range and the
NLOS_REPORTED flag change for affected measurements; identity, effective stamp
and Gaussian variance remain unchanged. Values above 800 m are not clipped.
The original MCAP remains immutable; profile SHA plus the original source
identity identifies the derived input, and delivered hashes must agree between
producer, observer and paired runs. Audits recompute the transform, check
estimator-observed values and retain navigation errors separately.

### User-authorized raw capture release

The user authorized bounded campaign storage on 2026-10-21. This is an explicit
exception to protected raw-capture retention, not ordinary run garbage collection.
After all three modes pass their per-run audits and strict pairing is saved,
the nominal campaign releases only MCAP files for seeds other than 0. Seed 0
remains a representative complete input. Failed/incomplete groups are not
released automatically. The three separately authorized historical captures
are listed in the phase-two acceptance report.

Before deletion, `metadata/manifests/raw_capture_release.json` records each
relative path, byte count, SHA256, reason and a prepared status. Successful
deletion marks it released; interrupted release requires inspection. Only direct,
nonsymlink, terminal project runs qualify. Primary manifests and original capture
hashes remain immutable historical records. Reports, failure evidence, referenced
images, configuration, source/binary identities, trajectories and diagnostic CSV
remain. Replay is unavailable after release; later CSV/identity re-audits do not
constitute a new verification of deleted MCAP bytes. Do not substitute regenerated
seed data for the historical frozen input.

Campaign audit additionally reports `independent_run_summary`: equal weight per
run, explicit seed/run/value lists, unavailable values, mean, min/max and sample
standard deviation when at least two runs exist. Frame counts do not weight
these statistics. Covariance NEES/coverage remain descriptive per-run diagnostics,
not pooled-frame chi-square tests. Summary means never replace per-run gates;
missing seeds and individual failures retain campaign FAIL.


Deployment v4 supersedes dense-layout acceptance: seven anchors at longitudinal
0/400/800/1200/1600/2000/2400 m, at most three in any closed 1000 m interval.
Map generation, owner preflight and deployment audit reject density violations;
formal replay rejects historical dense input identities. Historical files/results
are not rewritten. A density-compliant layout alone does not prove observability.

`metadata/manifests/watchdog.json` binds wall and no-progress limits and the two
progress files. Simulation and estimator output growth are monitored separately;
process liveness alone does not count as progress. Watchdog failure follows owned
child teardown and retains input_failure. Evaluation has a separate finite limit.

S3 adds `postfit_ta_stamp,postfit_ta_count,postfit_ta_squared_sum,postfit_ta_limit`
to trajectory CSV and the count/sum/limit to diagnostics. Only newly inserted
physical TA factors are evaluated at the current joint optimized estimate;
sum is twice their Gaussian factor error. The reference is chi-square(n) 99.9%.
Exceedance reports LOST/UWB_POSTFIT_INCONSISTENT; invalid values report LOST.
This does not alter optimization or covariance. It is a post-fit disagreement
diagnostic, not predictive NIS or calibrated nonlinear false-alarm probability.
Small residuals cannot certify global accuracy in weak geometry. No new TA keeps
the last check; missing input takes precedence; session reset clears the check.

S13 appends `failed_recovery_stamp` to trajectory CSV and diagnostics (`-1` when
unset). A new TA post-fit conflict/invalid value within a declared recovery lag
window records the first failure. Subsequent small residuals cannot clear it:
otherwise healthy output is DEGRADED/UWB_RECOVERY_UNVERIFIED. Fresh conflicts
remain LOST; missing input and invalid posterior retain their specific reasons.
This is a conservative failure report, not an accuracy certificate or an automatic
relocalization algorithm. The current implementation clears it only when the
estimator resets all graph/session state and initializes a new session. It never
resets automatically in response to this flag. Normal uninterrupted fusion and
recovery windows without a detected conflict do not latch. State, factors and
covariance are not modified by this reporting rule. Automatic verified recovery
remains an unmet phase-two capability.

S15 adds `gradient_rx,gradient_ry,gradient_rz,gradient_tx,gradient_ty,gradient_tz`
and `gradient_world_x,gradient_world_y,gradient_world_z` to icp.csv. These are
summed current-X cost gradients from the already computed raw ICP linearization
at the pre-update prediction. Pose3 components use right-local rotation/translation;
world translation gradient is rotated using the predicted body attitude. Positive
world-x gradient means negative x is a local descent direction, not a physical
force or proof of causation. No extra factor evaluation/linearization is performed.
Profiling odometry.csv adds `pred_vx,pred_vy,pred_vz` (world m/s) from IMU propagation
before this update. Changes in prediction versus optimization can be decomposed
algebraically but remain coupled through prior updates; they are not independent
sensor-error contributions.
