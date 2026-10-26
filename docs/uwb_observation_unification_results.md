# UWB interface unification — work in progress

Started 2026-10-26. **Not accepted; Goal remains active.** Historical phase-two
and phase-three results and failures are unchanged.

Baseline LIUO: clean master `1330b6e3fb1b6cc536a04a4833bd36355c4738f5`.
Read-only driver: `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`.
No new branch/worktree, no push. Data root for builds and test evidence:
`/home/dev/ws_iap/liuo_observation`; run root is supplied by data-run
(`/home/dev/ws_iap/.runtime/0/liuo` on this host).

## Evidence so far

All artifact-generating commands use `data-run --`. `data-run --check` passed
Landlock ABI 7 and data-mount verification. nvidia-smi found an RTX 4070-class GPU
with driver 580.126.09; unrelated graphics/compute load was present and untouched.
This is device preflight only, not GPU regression acceptance.

- `tools/build.sh`: isolated message underlay plus three LIUO packages built on
  the data disk. `evidence/m0-build.log`, `m1-build-2.log` passed. Initial M1 build
  failed on run-info string typing; `m1-build.log` retained.
- `test_observation_input`: 3 C++ tests passed, covering peer identity, wrap,
  retransmission, reorder retirement, unknown evidence, invalid fields, source
  restart and first-available AA reporter. `evidence/input-unit.{log,xml}`.
- `tools/check_pressure_message.py`: actual generated ROS CDR/hash round trip,
  NaN preservation, float32 distance-only stress passed in
  `evidence/pressure-message-2.log`. First fixture incorrectly compared a Python
  double with its float32 serialized value; failure retained in pressure-message.log.
- `python3 -m unittest discover -s tools -p 'test_*.py'`: 57 passed in
  `evidence/python-tests-3.log`. Earlier attempts found a source-tree temporary
  path, missing explicit message environment and stale delivery fixtures; retained.
- Targeted CTest initially passed the input tests and 20/21 existing UWB tests;
  the remaining test wrote a hardcoded source-tree build path. Failure is in
  `evidence/uwb-ctest.log`; temporary-path repair is being verified.
- First capture attempt was correctly refused before execution due to uncommitted
  work: `evidence/capture-smoke.log`, failed run `20261026T130138Z_616`.

## Acceptance matrix

T01 partial (build/type; live QoS pending). T02–T07 partial component evidence.
T08 graph integration pending. T09 actual capture/replay pending. T10 partial
serialization/scheduler evidence; ROS reset integration pending. T11 production
input integration pending. T12 60-second load pending. T13 representative GPU
simulation and physical baseline comparison pending. T14 independent map reuse
pending. No item is marked fully passed from component evidence alone.

Remaining work includes full topic/QoS propagation, bounded observer audit state,
complete raw identity and graph-count assertions, public ROS fixtures and load,
float32 physical-equivalence capture, representative self-calibration/export/reuse,
README final commands and final clean revision/evidence binding. No real wireless,
hardware clock or deployment claim is made.

## First committed live smoke evidence

Implementation slice `bd2572a`; clean-runtime fixes `46a6306`, `59de051`.
These are intermediate commits, not final acceptance revisions.

- Targeted CTest after temporary-path repair: input adapter, existing UWB suite,
  calibrated-map suite all passed (`evidence/uwb-ctest-2.log`).
- GPU preflight failures `20261026T130616Z_078` and `20261026T130706Z_057`
  reported cuInit=304. Direct data-run initialization succeeded. Diagnosis showed
  env isolation discarded the launcher's thread-name broker and preload, needed
  for CUDA's procfs thread naming under Landlock. Preserving that existing broker
  fixed preflight; storage protection was not changed. Cache/ROS directories are
  also preserved. The failed logs are capture-smoke-2.log and capture-smoke-3.log.
- Clean commit `59de051`, capture run `20261026T130803Z_515`: 6 s, 1200 IMU,
  1200 clocks, 60 clouds, 384 batches/observations/unique reports, zero violations,
  raw_input_valid=true. Actual MCAP retained under export/capture/raw. Command:
  `data-run -- tools/run.sh --duration 6 --capture-only --capture --wall-timeout 60`.
  This run did not execute localization.
- Replay run `20261026T130841Z_172` with default phase1 configuration failed on
  its pre-existing missing max_keyframes setting; error retained. The documented
  default 15 must be used consistently by the matching code.
- Replay run `20261026T130927Z_930` with phase2_continuous_walls_off.json consumed
  the same captured input through the production callback, produced 39 DEGRADED
  states, and verified all raw hashes/counts. Evaluation FAIL solely because the
  new input metadata write replaced the earlier GPU identity fields. Consolidating
  those fields fixes provenance; this failed run is not relabeled PASS.
  Command: `data-run -- tools/run.sh --replay
  /home/dev/ws_iap/.runtime/0/liuo/20261026T130803Z_515 --config
  packages/liuo_tunnel/config/phase2_continuous_walls_off.json --wall-timeout 60`.

Next: rebuild and repeat the short replay after metadata/default correction;
then public batch fixtures with malformed/mixed/driver-like inputs, 60-second
load with per-edge/window accounting and resource assertions, full topic/depth
propagation, and 360-second representative calibration plus independent reuse.
Physical equivalence must retain exact pre-float32 generated samples and mapped
identities; same seed alone is not accepted. Current adapter component tests are
not a substitute for graph-side assertions in those runs.
