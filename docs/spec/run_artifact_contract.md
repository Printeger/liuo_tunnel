# LIUO Run Artifact Contract

Status: normative
Schema: `liuo_tunnel_run_artifact_v1`

This document is the sole authority for artifacts produced by an LIUO run.
`AGENTS.md` contains only enforcement rules; user-facing READMEs contain only
commands and navigation.

## 1. Terms and ownership

- **run root**: the directory configured by `LIUO_RUN_ROOT`.
- **run ID**: a collision-safe UTC identifier of the form
  `YYYYMMDDTHHMMSSZ_mmm`, optionally followed by `_NN`.
- **run directory**: exactly `<run root>/<run ID>`.
- **run owner**: the top-level launcher, or `RunLogManager` for a standalone
  process. Exactly one owner allocates and finalizes a run.
- **adopter**: a child process that receives `LIUO_RUN_DIR` and writes only
  inside that run.
- **automatic artifact**: any output created as a side effect of running LIUO.
- **explicit export**: a user-requested `save(path)` or offline conversion to
  a caller-selected path.

An automatic artifact MUST be contained by the current run directory. An
explicit export MAY be outside it, but the owner MUST record its absolute path,
kind, requester, and timestamp in the run manifest. Configuration defaults do
not count as explicit user intent.

## 2. Public configuration and allocation

For sustained diagnostic campaigns, retain the unique frozen input bytes and
their source/configuration/hash binding, every requested sample's availability
and failure ledger, selected physical observations needed by the declared
experiment, and baseline/candidate/final results including failures. Metadata
alone does not replace `input.bin`. Repeated derived exports and full-frame raw
traces are not routine deliverables; large raw capture requires a specific
declared question and remains opt-in. Stream lossless compression for such
traces rather than keeping expanded and compressed copies. Sample compaction
must record selection and missing data, preserve the original denominator and
all acceptance gates, and must not claim full-trace reproducibility. This policy
does not authorize ordinary retention to delete protected or active runs.

`LIUO_RUN_ROOT` is the only user-facing persistent run-location setting. In a
source or symlink-install workspace it defaults to `<liuo_tunnel source>/log`; in an
installed package it defaults to `${XDG_STATE_HOME:-~/.local/state}/liuo_tunnel/log`.

`LIUO_RUN_DIR` is an internal, absolute leaf-directory contract passed by a run
owner to child processes. A process receiving it MUST adopt that exact run and
MUST NOT allocate another timestamp or update `latest`.

The owner creates the run with atomic `mkdir`. Collisions append `_01`, `_02`,
and so on. Allocation and `latest` updates are serialized by a lock in the run
root. `latest` is an atomically replaced relative symlink. A non-symlink named
`latest` is an error and MUST NOT be removed.

The former user-facing launch argument `output_dir` has been removed from all
four canonical entrypoints. New tools and documentation MUST use
`LIUO_RUN_ROOT`; `run_dir` exists only for private owner-to-child adoption.

## 3. Required layout

Only the following four first-level directories are permitted:

```text
<run root>/<run ID>/
├── runtime/
├── profiling/
├── export/
└── metadata/
```

The canonical layout is:

```text
runtime/
├── liuo_tunnel_*.log
└── ros/

profiling/
├── liuo_tunnel_timing.csv
└── planner_timing.csv

export/
├── glio/
├── current_integrity/
├── advisory/
├── planner/
├── simulation/
├── capture/
└── analysis/

metadata/
├── run_manifest.json
├── config/
├── processes/
└── manifests/
```

GNSS factor, ICP, estimator trajectory, map, and factor dumps belong to
`export/glio/`. Current ARAIM and PL/AL/IM evidence belongs to
`export/current_integrity/`. Future-integrity and risk-grid evidence belongs to
`export/advisory/`. Candidate, P1-P5, lineage, certification, and HOLD evidence
belongs to `export/planner/`. Simulation truth and metrics belong to
`export/simulation/`; rosbags to `export/capture/`; offline reports and plots to
`export/analysis/`.

`ROS_LOG_DIR` MUST be `<run>/runtime/ros`. Effective and source configuration
snapshots MUST be under `metadata/config/`. No CSV or manifest may be written
directly in the run directory.

## 4. Manifest and lifecycle

`metadata/run_manifest.json` is the only run-level authority. The owner writes
it atomically using a temporary sibling and rename. It contains at least:

- `schema_version`, `run_id`, `run_root`, and `run_dir`;
- `entrypoint`, optional `scenario`, and enabled `modules`;
- `started_at_utc`, nullable `ended_at_utc`, and `lifecycle`;
- `safety_outcome`;
- source commit and dirty state, build identity, and host identity;
- configuration snapshot and subordinate-manifest references;
- `run_class`, `retention_class`, and `external_exports`.

Lifecycle values are `active`, `completed`, `failed`, and `interrupted`.
Safety outcomes are `executable`, `hold`, `not_applicable`, and `unknown`.
HOLD is a valid safety result and is not a process failure.

Only the owner updates the run manifest. Children write process records under
`metadata/processes/` or immutable subordinate manifests under
`metadata/manifests/`. The historical `metadata/run_info.json` is a read-only
input format for migration tools and MUST NOT be generated for new runs.

## 5. Path API rules

Artifact APIs accept a category/module and a relative leaf name. They MUST
reject absolute names, `..`, and any resolved path outside the run. Modules
MUST NOT create their own timestamp directory, infer a root from cwd, or honor
the directory component of a legacy `*_csv_path` setting. During migration a
legacy path may contribute only its basename and MUST produce one warning.

Machine-specific paths and `/tmp` output defaults are forbidden in maintained
configuration and production writers. Input resources are configured
independently and are not artifacts.

## 6. Analysis and compatibility

`tools/ana_log.py` defaults to `<source>/log/latest`. New analysis artifacts
are written only below `export/analysis/` and MUST NOT overwrite online module
outputs. It may read the historical flat `export/*.csv` layout and
`metadata/run_info.json`, but compatibility reads do not authorize new writers
to produce those layouts.

## 7. Retention

Before allocating a new ordinary development run, the owner may prune old
runs. The repository-local default root enables this automatically. A root
selected through `LIUO_RUN_ROOT` outside the repository requires
`LIUO_RETENTION_ENABLED=1`.

Retention keeps the newest three finalized runs and every run ended within the
last seven days. A run is eligible only when it is both older than seven days
and outside the newest three.

The collector examines only direct, non-symlink children whose names match the
run-ID grammar and whose manifest says `run_class=development`,
`retention_class=ordinary`, a terminal lifecycle, and a non-null end time. It
MUST skip active/locked, malformed, formal-evidence, protected, symlinked, and
unrecognized directories. It MUST never traverse into `results/` or delete an
explicit external export. Under the root lock it revalidates the candidate,
atomically renames it to a root-local quarantine name, and only then removes
the quarantined tree. A failure is reported and does not block allocation.

A protected campaign owner may record `retained_input_run_ids` in its own
primary manifest. The collector also skips those named direct-child runs under
the same root, including while the protected owner is active. References must
be a list of valid run IDs; malformed/symlinked owners and other roots do not
provide pins. This preserves a campaign's unique inputs and failed results
without rewriting their already hash-bound primary manifests. The campaign
owner alone sets its protection and evidence references; child manifests do not
change run-level retention authority. The owner establishes protection/references
under the root retention lock before publishing the campaign. Ordinary
unreferenced retention is unchanged.

## 8. Migration

Migration proceeds in this order: freeze this contract; adopt the outer run in
C++; normalize GLIO/simulation/flight categories; contain ROS logs and
manifests; migrate analyzer and READMEs; remove temporary and machine paths;
then enable retention. A checked allowlist may temporarily describe existing
violations, but new violations are forbidden and the allowlist must be empty at
the end of migration. Formal or frozen evidence is never rewritten to look
like a new run.


## Tunnel scope

Production exports use odometry/, simulation truth uses simulation/, raw capture uses capture/, offline evaluation uses analysis/. No advisory, planner or GNSS modules exist. Acceptance is independent of lifecycle: not_evaluated/pass/fail/incomplete. Phase-one UWB has no graph keys or innovations; log it as RECORD_ONLY.
