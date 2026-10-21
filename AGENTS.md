# LIUO Tunnel rules

- Modify project files only here; IAP and demo are read-only provenance sources.
- Inspect branch and worktree before edits. Preserve user changes. No extra branches or worktrees, history rewriting, or broad process cleanup.
- Stage only task files; commit checked logical changes. Formal live evidence binds a clean committed revision.
- Phase one uses original CPU VGICP factors and IMU in a fixed-lag X/V/B graph. UWB is recorded only. No predicted-pose priors, GNSS, clock states, planning, or duplicate motion observations.
- Truth belongs only to simulation and evaluation. Never subscribe to truth in the estimator or constrain it to the rail.
- New abstractions, states and weighting need a reproduced problem. Resolve the first violated invariant, without silent fallback or weakened acceptance.
- Coordinate frames, units, timestamps and covariances are contracts; update callers and meaningful tests together.
- One LIUO_RUN_DIR per run; shared resolver only. Artifacts live in runtime, profiling, export, metadata. ROS_LOG_DIR is runtime/ros. Owner alone finalizes the atomic manifest. Preserve failures, active/protected runs and external outputs.
- README describes commands; docs/spec contains contracts; handbook describes phase scope. Record tests and evidence honestly. Zero exit and RViz are not acceptance.
- Retain copied licenses and provenance. Use the provided 2-space/180-column formatter.
- GPU requires driver/device preflight and separate evidence; CPU success never implies GPU success. Snapshot does not validate rolling/deskewing.
