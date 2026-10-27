# LIUO Tunnel

统一 UWB 接口使用 `liuo_uwb_msgs/msg/UwbObservationBatch`，默认
`/liuo/uwb/observations`、best effort/volatile、depth 100。当前验收进展与失败见
[接口统一结果](docs/uwb_observation_unification_results.md)：T01–T14 已通过声明范围的软件集成、
负载及代表性 GPU 仿真融合/地图复用验收；不扩展为硬件或全场景部署认证。

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
data-run --check
data-run -- tools/build.sh
data-run -- tools/test.sh
# 默认 GPU 仿真；配置和产物由 owner 写到数据盘。
data-run -- tools/run.sh --duration 6 --capture-only --capture --wall-timeout 60
# 自定义 topic/depth 贯穿 simulator、生产节点、observer、capture/replay 和可视化。
data-run -- tools/run.sh --duration 20 --uwb-topic /experiment/uwb --uwb-qos-depth 37
# 首次未知基站建图：仅无坐标安装资料，显式开启 AA。
data-run -- tools/run.sh --duration 360 --seed 0 --sim-start 200 0 1.2 0 --calibration-route --capture --config packages/liuo_tunnel/config/phase3_self_calibrate.json --installation packages/liuo_tunnel/config/installation/straight_simulated.json --use-aa --formal --wall-timeout 480
# 用打印的首轮路径替换 FIRST_RUN；独立起点/朝向/噪声。
data-run -- tools/run.sh --duration 360 --seed 1 --sim-start 220 .2 1.6 1.5707963267948966 --calibration-route --capture --config packages/liuo_tunnel/config/phase3_reuse_map.json --map FIRST_RUN/export/map/calibrated_map.json --use-aa --formal --wall-timeout 480
```

构建使用 `tools/environment.sh` 中的数据盘 build/install 和唯一消息 underlay。
AA 默认仍关闭；`--use-aa` / `--no-use-aa` 仅显式覆盖当前运行配置。
下面的旧报告和 log 路径属于历史证据；当前命令的输出不能写入源码树。

TA 已作为基础能力，AA 默认关闭，可通过 `uwb.use_aa: true` 启用。[最新 TA-only 配对报告](docs/aa_optional_comparison.md)复用此前其他方法结果；新增无图运行存在漂移，有图初始化失败，包含完整图表和失败证据。

最新空隧道入口启动对比（移除钢轨、枕木、柜体）：[FAST-LIO / 无图 / 有图的 APE、XYZ、三维轨迹与 RPY](docs/empty_tunnel_comparison.md)。主指标使用固定原始坐标约定，不拟合轨迹；保留初始化失败和有图漂移证据。

五组同输入精度实验已保存为[图文 APE 对比报告](docs/ape_five_method_comparison.md)：FAST-LIO2、LIO-SAM 六轴适配、本方法无图/有图 UWB、本方法 UWB OFF。包含共同时间 APE、分轴误差、轨迹、覆盖率、失败状态和可复现命令；不是全面部署或算法普遍排名。

第三阶段已按代表性范围交付：**代表性仿真中首次自标定及独立地图复用有效**。完整360秒首轮/次轮、四站联合地图、同输入ON/OFF/已知地图参照及AA消融见[交付报告与完整命令](docs/phase3_representative_results.md)。地图高度和局部后验仍弱，不是全面部署认证。

第二阶段按用户修订范围完成，结论仅为**代表性仿真范围内系统完成且验证有效**。使用原始GPU VGICP、IMU和UWB同图紧耦合；三个内部起点、冻结输入全程ON/OFF及5s/20s断流已有证据。正式配置、命令、run路径与限制见[交付报告](docs/phase2_representative_results.md)，历史失败见[逐轮记录](docs/phase2_acceptance.md)。

正常融合位置RMSE1.034m，高度仍弱；20秒断流仍有明显速度/位置漂移，尚不能认证恢复。局部协方差过度自信未解决。0.30m等数值保留为参考目标，未放宽噪声或伪造信息。单次PASS不是阶段或部署认证；未验证任意启动、完整无人机、真实Mid-360/rolling、十种子及全面部署。

部署条件以[用户确认合同](docs/spec/deployment_initialization_contract.md)为准：没有固定起点/高度/航向；已知外参和静止启动可用。第二阶段为已知不确定地图实验；第三阶段自标定/复用仅通过声明的代表性验证，不能将局部结果称为全面部署通过。Independent ROS 2 Jazzy project; phase one and representative phase two use the documented GPU snapshot scope.

以下为基础构建及历史工具入口；当前第二阶段请使用交付报告中的冻结配置与原包命令。

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
data-run -- tools/build.sh
data-run -- tools/test.sh
data-run -- tools/run.sh --formal --duration 45 --seed 0
data-run -- tools/run.sh --duration 20 --reset-at 10 --seed 0
data-run -- tools/run.sh --duration 20 --inject-failure odometry
data-run -- python3 tools/evaluate.py /home/dev/ws_iap/.runtime/0/liuo/<run_id>
```

Build/run wrappers discard inherited overlays, source `/opt/ros/jazzy`, and then load only this project's installation. Build scans only `packages/`. Dependencies: ROS 2 Jazzy, Eigen, PCL common, GTSAM, gtsam_points, spdlog, Boost filesystem, OpenMP. The required CUDA-enabled gtsam_points 1.2.1 uses public GPU binary matching factors and the extended incremental smoother. A working NVIDIA device/driver is required for the default configuration and GPU tests. Formal runs require GPU and requested rate 1.0; failures never switch to CPU. An explicit alternate configuration with odometry.backend=cpu is diagnostic only.

Run artifact-generating commands through `data-run --`. Artifacts belong on `/home/dev/ws_iap`; `LIUO_RUN_ROOT` accepts an explicit absolute data-disk override. Each live run has one owner, atomic `metadata/run_manifest.json`, copied configuration, source and binary identity, child console/spdlog logs, CSV observation/trajectory/graph/information/timing exports, and offline `export/analysis/report.json`. Failures remain in their original runs. The launcher rejects dirty source before live evidence. SIGINT stops only its own children. Tests use automatically cleaned temporary directories. Build/test tooling outputs use `$LIUO_TOOL_LOG`, separate from simulation run IDs.

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
export ROS_LOG_DIR="$PWD/log/launch"
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
# RViz 启动入口

在本项目目录执行（ROS 2 Jazzy，已有 GPU 依赖）：

```bash
./tools/build.sh
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export LIUO_RUN_ROOT="$PWD/log"
export ROS_LOG_DIR="$PWD/log/launch"

# 第一次：无基站坐标地图，联合估计轨迹、基站及安装参考
ros2 launch liuo_tunnel first_mapping.launch.py

# 第二次：显式指定第一次输出的地图；不自动猜测“最新地图”
ros2 launch liuo_tunnel reuse_map.launch.py \
  map:="$PWD/log/<第一次运行目录>/export/map/calibrated_map.json"
```

两个入口默认打开 RViz，运行 360 s 的代表性仿真路线，使用 GPU。第一次地图初始化可能需约 50 s；初始化未完成前不显示虚构的全局点云或基站。原始测距数值可先显示，当前帧点云在有效估计输出后显示。默认起点仅传给仿真器：首次和复用均为入口 `0 0 1.2 0`。当前场景为无钢轨、无枕木、无柜体的空矩形隧道（empty_tunnel_straight_v2），历史结果仍对应各自冻结场景。

地图路径会随本次运行打印的 run 目录确定。首次输出是带相关协方差的 **UWB 基站/安装参考地图**；未初始化的基站不在已知坐标中。导出的 provisional 地图及 DEGRADED 状态不等于部署认证。

RViz 显示当前帧（降采样 snapshot 点云）、点云累积地图、当前图内的基站估计，以及接收的原始测距和状态。点云累积仅用于显示：0.4 m 体素、最多 150,000 点、旧点淘汰、不做历史位姿重优化，不作为第二次定位的先验，也不是另一个已保存的稠密 SLAM 地图。基站橙色球及文字是估计，不是真值。测距连线用最近 60 ms 内估计位姿和当前基站估计绘制，数字是原始接收值；连线长度不等于原始测距，接收也不等于被优化器接受。缺少端点时仅显示数字。

可用参数：`rviz:=false`（仍发布显示话题）、`duration:=80`、`wall_timeout:=180`、`capture:=true`、`formal:=true`、`seed:=0`、`sim_start:="0 0 1.2 0"`；`replay:=/绝对路径/已有run` 复用冻结输入。正式证据仍要求干净提交。已有命令行入口 `tools/run.sh` 保留，可加 `--rviz` 或 `--visualize`。所有进程在同一 run 和隔离的 ROS domain 下，由运行器统一收尾；关闭 RViz 提前退出会记录为中断/失败，不静默继续。Ctrl+C 停止本次运行。

RViz 固定坐标系是 `map`。可在 Views 调整视角，或选中点云按 F 聚焦；长路线不会自动追随机器人。桌面运行需要有效 DISPLAY/Wayland 会话。仅查看原始传感器帧可在 RViz 手动添加 `/sensors/lidar/points`，将 Fixed Frame 改为 `lidar`；该模式不能同时对齐全局地图，初始化前没有合法的全局变换。

### TA 基础与可选 AA

`uwb.use_aa` 默认 `false`：TA 与 LiDAR/IMU 紧耦合保持开启；设为 `true` 才加入新的基站—基站 AA 信息。缺省字段同样按关闭处理，必须使用 JSON 布尔值。原始 AA 仍可接收和记录，关闭时标记 `AA_ABLATION_DISABLED`，不进入初始化、扩图、滑窗或 AA 摘要。已有地图内吸收的 AA 不能通过这个开关移除；完整 TA-only 复用需由 TA-only 首次运行生成地图。历史 AA 开启实验在新版复现时须显式 `use_aa=true`。本轮协议见 [AA 可选能力验证](docs/aa_optional_comparison_protocol.md)。

### UWB batch migration (in progress)

The current development input is `liuo_uwb_msgs/msg/UwbObservationBatch` on
`/liuo/uwb/observations`. See [input contract](docs/spec/uwb_observation_input_contract.md)
and [current validation status](docs/uwb_observation_unification_results.md).
The bundled noise/time models are explicitly simulation assumptions.

Build and test using the protected data-disk environment:

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
data-run --check
data-run -- tools/build.sh
data-run -- tools/test.sh
```

The default build/install/message underlay lives at
`/home/dev/ws_iap/liuo_observation`. `LIUO_DATA_ROOT`, `LIUO_BUILD_BASE`,
`LIUO_INSTALL_BASE`, `LIUO_MSG_BASE` and `LIUO_TOOL_LOG` are explicit overrides
which must remain on the data disk. The clean environment loads Jazzy, that
message underlay and that target install. Existing historical bags are immutable;
new replay rejects UwbRange bags with an instruction to use the historical version.

## 7 km 道路场景：只生成数据，不运行定位

新入口与旧 rail_sim、capture-only、baseline runner 分开。参数及预声明容差在
`packages/liuo_tunnel_sim/road_scene/scene.json`，坐标、时间、安装偏距与信息隔离见
[场景数据合同](docs/spec/road_tunnel_scene_data_contract.md)。依赖 ROS 2 Jazzy、现有只读
`liuo_uwb_msgs` 安装，以及系统 NumPy/SciPy/Matplotlib；新增 mesh/Embree 依赖安装到数据盘。

从本仓库执行（所有产物留在 `/home/dev/ws_iap`）：

```bash
data-run --check
data-run -- timeout 180 tools/build_road_scene.sh
# 独立几何、运动、网格收敛及接口夹具；开发时允许未提交源码
data-run -- timeout 120 tools/road_scene.sh --check --allow-dirty
# 高速短片段：原始 MCAP、200 Hz 真值、离线检查、图、C++ 无估计订阅检查
data-run -- timeout 120 tools/road_scene.sh --start 180 --seconds 10 --allow-dirty
# 全程生成/直接录包：要求工作树干净；不等待定位订阅者
data-run -- timeout --signal=TERM --kill-after=20 7200 tools/road_scene.sh
```

每次输出一个 `LIUO_RUN_DIR`。唯一 owner 完成 manifest；失败记录不会被覆盖。
`export/capture/raw/` 是可回放 MCAP，只有原始 LiDAR/IMU/UWB/clock；
`export/simulation/` 的轨迹、站点、网格、设施是独立仿真真值。
`metadata/config/installation_metadata.json` 只含身份、侧别和 synthetic 横向偏距定义。
所有最终原始包和必要真值保留为后续实验输入。

`export/analysis/` 包含路线平面/三维、断面、场景点云、运动、UWB 覆盖 PNG，以及
`bag_audit.json`、`wire_check.json`、几何/运动检查和生成速率。点云图是明确标记的
仿真真值可视化，不是发布给算法的点云。预览可直接打开这些文件。

纯传感器回放/检查（无需读取 truth/config；将 RUN 改为实际目录）：

```bash
data-run -- bash -c '
  source /opt/ros/jazzy/setup.bash
  source /home/dev/ws_iap/uwb_ros2/install/setup.bash
  export PYTHONDONTWRITEBYTECODE=1
  export LIUO_RUN_DIR=/home/dev/ws_iap/.runtime/0/liuo/RUN
  export ROS_LOG_DIR=$LIUO_RUN_DIR/runtime/ros
  python3 tools/road_scene_io.py "$LIUO_RUN_DIR/export/capture/raw" \
    --seconds 2 --cpp /home/dev/ws_iap/road_scene_build/road_scene_wire_check \
    --report "$LIUO_RUN_DIR/export/analysis/wire_check_manual.json"
'
```

常规 ROS 回放可在同样 data-run/ROS 环境运行
`ros2 bag play "$LIUO_RUN_DIR/export/capture/raw" --rate 1.0`；原包已含 `/clock`，
不要额外生成另一份时钟。可用 `ros2 bag record` 仅记录四个原始话题到该 run 的
`export/capture/` 下新目录。不要启动任何旧定位 launch 来录制本场景。

未来 FAST-LIO2 标准 PointCloud2 接口所需的 native `ring:uint16` 与 `time:float32`
秒字段已经存在；`road_scene_io.native_fastlio_cloud` 只验证布局并原样传递，
不恢复 ring、不合成朝向、不去畸变。未来配置须显式使用秒时间单位、40 线、10 Hz、
本场景单位外参及无朝向观测 IMU。旧适配器仍是 snapshot-only，旧配置外参不适用；
本次不声称任何现有定位入口已支持此场景，亦不运行算法验证。
