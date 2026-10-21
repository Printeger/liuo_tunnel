# LIUO Tunnel

Independent ROS 2 Jazzy GPU LiDAR–IMU project. Phase one is accepted under the snapshot scope; phase two is in development and **not accepted**. The current known-map path adds raw UWB factors to the same GPU VGICP/IMU fixed-lag graph. See [phase-two results](docs/phase2_acceptance.md) for failures and remaining gates. The original phase-one 0.603× bottleneck has been repaired and short live runs reach 1×; this does not establish long-run fused accuracy or campaign acceptance.

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
./tools/build.sh
./tools/test.sh
./tools/run.sh --formal --duration 45 --seed 0
./tools/run.sh --duration 20 --reset-at 10 --seed 0
./tools/run.sh --duration 20 --inject-failure odometry
python3 tools/evaluate.py log/<run_id>
```

Build/run wrappers discard inherited overlays, source `/opt/ros/jazzy`, and then load only this project's installation. Build scans only `packages/`. Dependencies: ROS 2 Jazzy, Eigen, PCL common, GTSAM, gtsam_points, spdlog, Boost filesystem, OpenMP. The required CUDA-enabled gtsam_points 1.2.1 uses public GPU binary matching factors and the extended incremental smoother. A working NVIDIA device/driver is required for the default configuration and GPU tests. Formal runs require GPU and requested rate 1.0; failures never switch to CPU. An explicit alternate configuration with odometry.backend=cpu is diagnostic only.

All commands write artifacts under the project's ignored `log/`; `LIUO_RUN_ROOT` accepts an explicit absolute override. Each live run has one owner, atomic `metadata/run_manifest.json`, copied configuration, source and binary identity, child console/spdlog logs, CSV observation/trajectory/graph/information/timing exports, and offline `export/analysis/report.json`. Failures remain in their original runs. The launcher rejects dirty source before live evidence. SIGINT stops only its own children. Tests use automatically cleaned temporary directories. Build/test tooling outputs use `log/build` and `log/test`, separate from simulation run IDs.

Frames: W is simulator world; odom starts at IMU initialization with gravity alignment and yaw gauge zero. T_BL maps LiDAR into IMU/body. IMU has unavailable orientation and measures specific force. Output pose covariance converts GTSAM right perturbations into parent-frame ROS ordering. UWB antenna translation is T_BU; phase-one OFF uses no anchor coordinates; known-map fusion uses only its explicitly selected external map. Simulator truth is used by offline evaluation only; the observer audits estimator subscriptions.

`scan_model=snapshot` is explicit, with all point `time=0`. Rolling scan generation and IMU deskew are not validated and unsupported modes fail. The first-hit renderer returns world intersections; the simulator converts them into the actual LiDAR frame before adding range noise. The 2.4 km straight world is loaded in bounded local blocks; UWB LOS uses the entire convex corridor. Nonperiodic equipment exists only in the first 50 m; the rest contains repeated rails/sleepers and planar walls. Motion has continuous acceleration, 5 s initial standstill and intermediate stops at 600/1200/1800 m; a full run needs about 540 simulation seconds.

Phase-one VGICP factors connect the current frame to the last three variable poses and up to 15 active-window keyframes. The phase-two candidate connects the previous pose and the most recent distance/rotation keyframe (2 m / 0.5 rad selection, deduplicated when they coincide); both are original raw GPU factors within the same variable-state graph. This candidate passed the listed 180 s seed-0 thresholds, not the complete campaign. X/V/B timestamps use a 10 s lag. No fixed historical target or additional IMU-predicted-pose prior is used. The LiDAR information spectrum is diagnostic. Local covariance plane normalization requires eigenvalue ratios λ1/λ2 ≥ 0.1 and λ0/λ1 ≤ 0.1; otherwise covariance remains isotropic, avoiding invented normals on sparse scan rings/corners. This is an actual covariance change tested through the VGICP Hessian. There is no spectrum-based posterior pose weighting. UWB CSV reason `RECORD_ONLY` applies to the OFF configuration. Fused mode records associations and raw factor innovations, with static A variables and one correlated prior for surveyed maps; oracle maps keep anchors constant.

Current evidence supports only the configurations actually run, and does not certify rolling/deskew, hardware operation, UWB fusion, calibration, map reuse or planning.

Phase-two development commands (a completed process can still FAIL evaluation):

```bash
./tools/run.sh --formal --duration 20 --config packages/liuo_tunnel/config/phase2.json --map packages/liuo_tunnel/config/maps/oracle_map.json --capture
./tools/run.sh --formal --replay log/<captured_run_id> --config packages/liuo_tunnel/config/phase2.json --map packages/liuo_tunnel/config/maps/surveyed_map.json
./tools/run.sh --formal --replay log/<captured_run_id>  # default phase-one config: UWB OFF
```

Replay verifies complete source input counts and capture file hashes, uses the original measurement identities and timestamps at 1×, and verifies the received per-topic content hashes. The source trajectory need not pass accuracy for its complete raw inputs to be usable. Partial captures from failed processes are rejected. The replay producer is the sole clock/sensor publisher. Truth is copied for offline evaluation only. Known-map outputs use `map`, with initialization from the joint raw-observation graph, no true vehicle pose prior. The bundled surveyed map is explicitly a simulated survey with correlated uncertainty, not a field survey.

Joint X/V covariance is exported to `export/odometry/covariance.csv`; body linear
velocity covariance includes pose/velocity cross terms. Offline NEES and coverage
appear in the analysis report and currently reveal substantial overconfidence.
Angular velocity uses the SO(3) log increment between adjacent optimized poses,
divided by their interval, expressed in the current body frame. It is an interval
estimate, not an instantaneous gyroscope reading. Full 6×6 body twist covariance
retains previous/current pose and velocity correlations. Publication starts only
when two poses exist; before that the state is INITIALIZING. Runtime wire checks
are required alongside the component derivative tests.
The 180 s seed-0 captured run reaches 1× but fails 3D accuracy and truthful status;
see the acceptance record before treating this development path as deployable.

The current **unaccepted v3** simulation layout has 49 anchors at 50 m spacing,
cycling four wall/height combinations. This is a user-authorized replacement of
the original 13-anchor scene; thresholds and range/noise remain unchanged. Of
250 exchange slots/s, 225 prefer TA and 25 prefer AA; idle slots may be borrowed.
Requested 20 Hz TA/1 Hz AA rates may be delayed by the budget; inspect actual
per-link timestamps and `export/simulation/scheduling.csv`. Bundled maps are v3
fixtures. Historical v2 captures require their matching archived maps; do not
apply the new fixture to old observations.

Frozen transport fault example:

```bash
./tools/run.sh --formal --replay log/<180s_captured_run_id> \
  --config packages/liuo_tunnel/config/phase2.json \
  --map packages/liuo_tunnel/config/maps/surveyed_map.json \
  --delivery-profile config/delivery/long_transport.json
```

The declared profile delays/reorders/duplicates or omits UWB delivery while
preserving original payloads, identities and effective measurement timestamps.
The bounded scheduler fails on overflow. `delivery.csv` records actions; the
report separately checks unchanged sensor hashes, actual delivered UWB hashes,
unique graph ingestion and LIO continuity/recovery. These transport checks do
not replace accuracy, latency or state-credibility acceptance.

Optional offline geometry analysis (NumPy/SciPy):
`python3 tools/range_geometry_analysis.py log/<oracle_capture_run_id>`.
This explicitly assumes exact relative truth motion to probe range ambiguity;
it is isolated from the estimator and is not an online accuracy result.

For the required full outbound/backward-return resource test, use the explicit
simulation option `--return-trip --duration 1070` (with the phase-two config, map
and `--capture`). The train reaches 2350 m at 530 s, stops five seconds, reverses
without turning its body, and reaches the entrance at 1060 s. Default runs retain
the original outbound-only motion. This option has component continuity checks;
full live return acceptance must be recorded separately.

`profiling/resources.csv` samples estimator RSS, retained cloud frames/keyframes
and queues at 1 Hz. CUDA device memory is labeled as device-wide usage, not
process-attributed memory. Sampling overhead is included in processing timing.

Full-route numerical/continuity/resource audit (not complete stage acceptance):
`python3 tools/long_run_audit.py log/<full_run_id> --require-return`.
Its denominator uses declared initialization plus every subsequent 10 Hz epoch;
late output cannot shorten the denominator. Finite DEGRADED output is counted as
available and reported separately; accuracy is checked independently. It requires
all outbound stops and, when requested, actual reverse motion back to the entrance.
RSS trends are reported without claiming a probability guarantee or a bound on
other processes' device memory. Short runs explicitly fail full-route coverage.

Use `tools/compare_runs.py --strict-estimator-settings ...` for controlled ON/OFF
comparisons: besides identical raw bytes/counts it requires matching LIO settings
and estimator binary, and equal UWB settings between fused runs. Historical runs
with different matching settings remain useful input comparisons, not mode-only
causal comparisons. For the seed campaign, generate independent simulated survey
fixtures with `tools/generate_known_maps.py <directory> --survey-seed <integer>`;
the unchanged default is 67129. Oracle geometry and the declared correlated survey
covariance do not change when the survey random stream changes.


名义多种子证据使用 `python3 tools/campaign_audit.py RUN... --output log/campaign.json`
审核显式提供的 runs：seeds 0–9 每个必须有一组 OFF/oracle/surveyed，逐次完整
路线检查、原始输入内容配对、相同LIO配置和二进制核对；漏项或重复项明确失败，
不会自动挑选较好结果或用均值掩盖失败。OFF仍须满足相同输入和时延预算，但
不作为已知地图精度候选。该报告仅覆盖名义campaign，故障、生命周期、完整
返回和状态/协方差解释仍需对应独立证据，不能单独作为阶段完成证明。


`python3 tools/run_campaign.py` 串行执行名义 seeds 0–9：每个seed采集540 s完整
去程走停，再以冻结输入运行oracle和OFF；模拟测绘seed为67129+1000×sensor
seed。GPU任务不并发。任一逐次物理/精度/输入审计失败立即停止并保留失败，
不会自动重试挑选结果。`log/campaign_*/campaign_progress.json`记录命令、owner
PID、run和逐次审计；仅该脚本正常完成仍不表示第二阶段完成。`--seeds 0`可
执行单组诊断，但最终campaign审计仍会因其他seeds缺失而失败。

压力回放完成后运行 `python3 tools/pressure_audit.py log/<run>`，核查断流及
恢复状态并保留原始精度失败；此报告不替代名义种子配对验收。

单基站失效压力回放使用
`--delivery-profile packages/liuo_tunnel/config/delivery/single_anchor_loss.json`；
仅停止声明基站的链路，不改变测距数值。其实际验收进度见第二阶段报告。

名义campaign可通过 `python3 tools/run_campaign.py --completed-runs <surveyed-run>
<oracle-run> <off-run>` 显式复用已完成的种子三模式组，再运行其余seeds。
每个复用run必须重新通过长程审计、输入/配置/当前二进制核对，surveyed
测绘随机流也须匹配；不完整组、重复模式及失败run拒绝。原campaign失败
记录保留，新campaign单独记录复用来源。

Campaign在启动前检查预计产物空间，每5 s监测2 GiB保留空间；不足时
仅向自身创建的owner发SIGINT，等待其收尾并记录失败；磁盘不足不会触发历史产物清理。
预计空间为540 s捕获1.5 GiB或回放0.25 GiB，另加2 GiB保留量；这是
根据实测的启动预算，不保证其他进程不会消耗空间。

相关NLOS正偏压力回放使用
`--delivery-profile packages/liuo_tunnel/config/delivery/correlated_nlos.json`。
它生成明确标记的派生测距输入，并保留原始值和正偏日志；不同模式必须
使用同一源run及profile并核对实际投递哈希，不能只比较种子。

名义 campaign 按用户授权采用逐组原始包释放：每个 seed 完成 surveyed/oracle/OFF
运行审计、严格输入哈希与设置核对并保存配对报告后，删除该捕获的 MCAP。
seed 0 始终保留为代表性完整输入；其余 seed 的已完成复用组也执行此策略。
失败或不完整组不会自动释放。保留全部报告、图片、轨迹和诊断 CSV、配置、
源码/二进制身份以及输入哈希。每个释放记录位于
`metadata/manifests/raw_capture_release.json`，包含原文件大小、SHA256 和原因。
被释放的历史输入不能再直接回放；相同 seed 重生成也不冒充原始冻结输入。
已完成组可继续用保留的 CSV 与身份记录复核，但不声称重新核验已删除文件。

**当前部署约束覆盖历史密集布局说明：** 正式候选为 7 个基站，纵向
0/400/800/1200/1600/2000/2400 m；任意闭区间 1 km 最多 3 个。墙侧/高度
按 handbook 5.1 的四角循环安装假设。旧 49 基站结果仅作诊断，不能用于
部署验收。当前只允许单 seed 代表性调查，`run_campaign.py` 拒绝多 seed；
尚未完成稀疏完整路线 ON/OFF 与断流机制验证，不启动十种子 campaign。

运行 owner 的 `--wall-timeout` 默认 max(120, duration/rate*1.5+60) 秒；
`--no-progress-timeout` 默认 30 秒，分别监测仿真时钟日志和估计器处理日志
的增长。超时记录明确失败，只收尾自身子进程；SIGINT 15 秒后仍未退出则
仅 kill 该子进程并最多再等 5 秒。评估另限 120 秒，无自动重试。
