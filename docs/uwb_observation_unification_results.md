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
