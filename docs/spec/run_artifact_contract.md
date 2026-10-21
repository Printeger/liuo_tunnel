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
