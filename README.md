# LIUO Tunnel

第三阶段已按代表性范围交付：**代表性仿真中首次自标定及独立地图复用有效**。完整360秒首轮/次轮、四站联合地图、同输入ON/OFF/已知地图参照及AA消融见[交付报告与完整命令](docs/phase3_representative_results.md)。地图高度和局部后验仍弱，不是全面部署认证。

第二阶段按用户修订范围完成，结论仅为**代表性仿真范围内系统完成且验证有效**。使用原始GPU VGICP、IMU和UWB同图紧耦合；三个内部起点、冻结输入全程ON/OFF及5s/20s断流已有证据。正式配置、命令、run路径与限制见[交付报告](docs/phase2_representative_results.md)，历史失败见[逐轮记录](docs/phase2_acceptance.md)。

正常融合位置RMSE1.034m，高度仍弱；20秒断流仍有明显速度/位置漂移，尚不能认证恢复。局部协方差过度自信未解决。0.30m等数值保留为参考目标，未放宽噪声或伪造信息。单次PASS不是阶段或部署认证；未验证任意启动、完整无人机、真实Mid-360/rolling、十种子及全面部署。

部署条件以[用户确认合同](docs/spec/deployment_initialization_contract.md)为准：没有固定起点/高度/航向；已知外参和静止启动可用。第二阶段为已知不确定地图实验；第三阶段自标定/复用仅通过声明的代表性验证，不能将局部结果称为全面部署通过。Independent ROS 2 Jazzy project; phase one and representative phase two use the documented GPU snapshot scope.

以下为基础构建及历史工具入口；当前第二阶段请使用交付报告中的冻结配置与原包命令。

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
按 handbook 5.1 的四角循环生成仿真，不能作为首次自标定的高度或顺序输入。旧 49 基站结果仅作诊断，不能用于
部署验收。当前只允许单 seed 代表性调查，`run_campaign.py` 拒绝多 seed；
当前已完成代表性稀疏全程ON/OFF与断流行为验证，见交付报告；十种子campaign仍未启动。

运行 owner 的 `--wall-timeout` 默认 max(120, duration/rate*1.5+60) 秒；
`--no-progress-timeout` 默认 30 秒，分别监测仿真时钟日志和估计器处理日志
的增长。超时记录明确失败，只收尾自身子进程；SIGINT 15 秒后仍未退出则
仅 kill 该子进程并最多再等 5 秒。评估另限 120 秒，无自动重试。

Sparse S3 status diagnostics additionally check newly inserted TA factors after
optimization. Excessive measurement-sigma-normalized squared residual reports
`LOST/UWB_POSTFIT_INCONSISTENT`; the statistic and reference are in trajectory.csv.
This does not repair drift or calibrate covariance; small residuals do not prove
correct global position. See docs/phase2_acceptance.md for failed sparse evidence.

After a declared UWB recovery window, elapsed time and small residuals do not
restore TRACKING automatically; independent recovery verification is not yet implemented. `DEGRADED/UWB_RECOVERY_UNVERIFIED` and the recorded
`failed_recovery_stamp` (when a post-fit conflict occurred) persist until complete new-session initialization; the
estimator continues publishing its actual trajectory and covariance. This exposes
an unresolved recovery failure, not successful relocalization. No automatic reset
or truth correction occurs. Phase two remains incomplete while this recovery and
the documented sparse-layout accuracy failures are unresolved.

当前七基站安装候选为 v5 高位优先四角循环：纵向 0/400/800/1200/1600/2000/2400 m，(y,z) 循环 (2.8,4.6)、(-2.8,4.6)、(-2.8,0.4)、(2.8,0.4) m。数量、密度、量程、噪声和验收门槛不变。此候选已做实际 GPU 路线和代表性断流验证，精度与可信恢复仍 FAIL；历史 v4 冻结输入只能配其原地图快照，不能当作 v5 输入。


## 历史第三阶段局部复现（保留）

以下命令使用保留的冻结原始输入及实际首轮地图，不创建真值地图。首轮只加载无坐标安装资料，复用不再加载旧安装观测。输出地图位于本次run的`export/map/calibrated_map.json`，状态为PROVISIONAL_INITIALIZED；输入旧式完整场景来源的地图会被拒绝。

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
./tools/build.sh
export LIUO_RUN_ROOT="$PWD/log"
# 首轮 seed0 的同输入 TA+AA；TA-only替换为 phase3_self_calibrate_ta_only.json
./tools/run.sh --replay log/20261022T132941Z_415 \
  --config packages/liuo_tunnel/config/phase3_self_calibrate.json \
  --installation packages/liuo_tunnel/config/installation/straight_simulated.json \
  --formal --wall-timeout 120
# 独立 seed1 输入；下列地图是保留的真实首轮导出，可替换成上一步run的导出路径
./tools/run.sh --replay log/20261022T141659Z_884 \
  --config packages/liuo_tunnel/config/phase3_reuse_map.json \
  --map log/20261022T145334Z_197/export/map/calibrated_map.json \
  --formal --wall-timeout 120
```

评估器对校准地图使用首轮声明的固定坐标规范，复用时不按第二起点对齐；首轮来源run必须保留供离线评估。原始输入、地图及二进制身份在metadata/manifests内记录。单run PASS仅覆盖运行合同，不能证明地图高度可靠、失效已恢复或阶段完成。以上为旧局部入口；最新完整路线与冻结对照见第三阶段交付报告。snapshot不代表rolling或真实Mid-360通过。
