# 曲线与静态 AA 开发记录

状态：**开发中，C01–C16 尚未完成，不是算法验收报告**。

本 Goal 的明确授权来自 `curved_tunnel_static_aa_goal_prompt.md` 及活动 Goal 文件。已完成的纯场景任务与冻结数据保持原样。开发启动版本为干净 `master/5febe84`；并行设计文档已在 `6fd5d63` 提交，无待覆盖的用户修改。

## 已完成的开发检查

- `data-run --check` 通过，Landlock ABI 7，产物只允许写数据盘。
- L 原始 MCAP SHA256 已核对为 `05095c59adad36e908bfb802978e1ecc3ec3b63686755a03c081cb5d40b13005`；S 两份历史回归包仍存在。GPU 可见为 RTX 4070 Ti SUPER。此处只是预检，不是正式 1× GPU 算法运行证据。
- 代码确认旧 `append_ranges` 在区分 TA/AA 前应用车辆时间 bracket，导致迟到静态 AA 被过期窗口拒绝；现有地图仅有累计数，缺精确已吸收身份。这是新增静态账本的实现动机。
- 新 `StaticAnchorEdges` 复用原高斯摘要，实现有界物理叶身份、方向无关去重、失败前不提交的事务、已吸收叶账本编码/恢复。尚未接入 ROS 回调或生产地图导出。
- 项目内新增 `StaticAnchorRecord`/`StaticRangeLeaf`，ROS 消息生成构建通过。`StaticRecordInput` 的独立校验测试验证旧采集时间不会因车辆 lag 拒绝，覆盖不符和未配置来源拒绝；尚无生产订阅证据。

## 定向测试命令

从项目根执行（需要已有数据盘构建环境）：

```bash
data-run -- bash -c '
  source /opt/ros/jazzy/setup.bash
  colcon --log-base /home/dev/ws_iap/liuo_observation/log/static_messages build --base-paths packages/liuo_tunnel_msgs --build-base /home/dev/ws_iap/liuo_observation/build --install-base /home/dev/ws_iap/liuo_observation/install --executor sequential
  source /home/dev/ws_iap/liuo_observation/install/setup.bash
  cmake -S packages/liuo_tunnel -B /home/dev/ws_iap/liuo_observation/build/liuo_tunnel
  cmake --build /home/dev/ws_iap/liuo_observation/build/liuo_tunnel --target test_static_anchor_edges test_static_record_input test_uwb -j2
  /home/dev/ws_iap/liuo_observation/build/liuo_tunnel/test_static_anchor_edges
  /home/dev/ws_iap/liuo_observation/build/liuo_tunnel/test_static_record_input
  /home/dev/ws_iap/liuo_observation/build/liuo_tunnel/test_uwb --gtest_filter=UwbFactors.StaticPairSummaryPreservesRawCostGradientAndInformation:KnownMap.ReplacingStaticPairSummaryBoundsHistoryWithoutRepeatingMapPrior
'
```

开发采用先失败再实现：缺模块、缺事务更新接口、过长身份被错误接受、缺检查点接口、JSON 端点发生截断均有对应失败输出，随后修正。新的 6 个模块测试覆盖 1000 重复/反向交付、实际 GTSAM 缺端点失败及新求解器重试、容量/冲突/部署拒绝、身份长度、已吸收前缀增量和非法检查点。既有两个摘要/因子替换检查通过，未更改其容差。

这些测试中的坐标 prior 只用于构造可解的单元测试图，不是未知 Anchor 生产初始化的观测。

输入校验开发时曾尝试将加噪测量值 801 m 作为必拒测试，该用例失败后审查发现混淆了真实量程门限和高斯加噪观测。该错误单元假设已撤回，未增加观测值截断；真实 800 m 量程和 LOS 的生成端验收要求不变。这不是物理量程检查通过的证据。

## 仍需完成

生产静态消息与来源验证、TA/AA 生命周期分离、真实因子索引替换、未知端点延迟接入、原子联合检查点，以及曲线初始化、rolling、双场景完整首轮/独立复用、FAST-LIO2 对照、消融/交付等价与负载验证均待完成。模块 JSON 往返不能替代真实地图复用。当前不对任何 C 条款给出最终 PASS，不作定位效果结论。

## 生产接入实现进展

随后实现 `StaticEdgeGraph` 并接入生产静态订阅、2048 叶回调队列、未知端点等待、因子移除/替换、成功后记账及图重置后重新移交。真实 ISAM2 测试验证端点等待、1000 次重传不替换、增量替换后仅有一个 AA 因子；地图测试验证 A/L 后验和精确叶账本同文件原子往返、拒绝未知端点。旧 raw AA 的车辆 bracket 仅对 TA 生效；新静态通道避免与 raw AA 同时计权。

S 回归配置 `phase3_static_s_regression.json` 绑定旧 360 s 包 SHA256 `5215d2b35af02389e401f05c1925acb3c36707addc00700eff7aa47ef27ed26a`。回放派生器只处理送达的原始 AA，未读取物理全局日志；单元测试验证 driver 改变后物理 ID 不变、采集时间不改写、原包内容身份不符则拒绝。初次 Python 测试缺少共享消息 underlay，补齐与正式 wrapper 相同的环境后通过，没有绕过 data-run。

以上是实现与定向测试进展。实际生产回放、独立地图复用和完整 C01–C16 验收尚未完成；早先“待完成”条目仍是验收状态，不因新增代码自动标为 PASS。

### S 旧路线生产回归：`20261027T152225Z_083`

绑定干净提交 `14fbd3a`，实际 RTX 4070 Ti SUPER，360 s 录包在 360.00085 s 发布完成。输入 72,000 IMU / 3,600 snapshot / 26,614 UWB batch 全部核对，原始内容 hash 检查通过。命令：

```bash
data-run -- tools/run.sh --formal --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T031155Z_389 --config packages/liuo_tunnel/config/phase3_static_s_regression.json --installation packages/liuo_tunnel/config/installation/straight_simulated.json --use-aa --wall-timeout 480
```

run 路径 `/home/dev/ws_iap/.runtime/0/liuo/20261027T152225Z_083`；owner manifest 为 completed，`export/analysis/report.json` 单 run 合同 PASS，阶段验收仍 NOT_ASSESSED。实际未知 A 为 0/1/2，共接收 2,160 个独立静态叶，最终融合 720 个、两个显式 AA 因子，另外 1,440 叶因端点未初始化保留 pending。最后地图检查点在 t=360.0，含 718 叶；最终状态 t=360.9，比该检查点晚，因此不能把地图说成含全部 720 叶。其余 4 站未初始化；扩图日志保留未消除的位置分支。

首次输出 t=40.9（等待 39.9 s）。固定首轮规范的位置 RMSE 0.75249 m，分轴为 0.01760/0.02319/0.75193 m，高度弱误差未解决，未达到 0.30 m 参考值；输出 3197 个 DEGRADED 和 4 个 LOST。处理 p95 44.71 ms，端到端 p95 51.80 ms。该结果仅证明此旧路线上的生产静态接入与真实地图导出，不能替代完整 S/L、rolling 或独立复用。

另已确认旧 S 生成器只检查 AA 两端的量程/LOS，没有单独检查 reporter→车辆链路。旧包原样保留；本派生器只使用其中已送达的消息，但这不证明无线交付物理成立。新完整 S 采集必须补齐这项物理门限，旧回归不得作为 C10 的 LOS 通过证据。

独立复用的旧输入 `20261027T031819Z_304` 为 seed 1、启动 `[220,0.2,1.6,pi/2]`，与首轮 seed 0、`[200,0,1.2,0]` 不同。专用静态复用配置绑定其 raw SHA256 `5be10c3a8c89879865f0dc7c11d380d992ea429d51ddf6eca015c1e0a78764b4`；复用结果待实际运行，不按配置存在判通过。

### 旧 S 独立复用：`20261027T153032Z_460`

绑定 `117c96e`（相对 CAL 只有配置/文档新增，算法与实际节点二进制未变），真实新进程加载上述 v2 联合地图及其 718 个已吸收叶。命令：

```bash
data-run -- tools/run.sh --formal --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T031819Z_304 --config packages/liuo_tunnel/config/phase3_static_s_reuse_regression.json --map /home/dev/ws_iap/.runtime/0/liuo/20261027T152225Z_083/export/map/calibrated_map.json --use-aa --wall-timeout 480
```

run 为 `/home/dev/ws_iap/.runtime/0/liuo/20261027T153032Z_460`，owner completed，单 run 合同 PASS，完整输入核对通过，360 s 实际用时 360.00213 s。保持首轮坐标规范，首次输出 t=20.4，3406 输出中 3387 DEGRADED、19 LOST；位置 RMSE 0.85783 m，分轴 0.04299/0.03496/0.85603 m，处理 p95 43.57 ms。高度仍是主要弱项，不声称精确高度或统计一致性。

最终 3 个持久 A、一个联合地图 prior、零重复安装因子、两个新显式 AA 摘要，独立新输入实际融合 720 叶。原地图 718 叶只在 prior 中，未生成旧显式因子。已完成旧路线的真实未知 A→联合地图→独立复用小闭环；不替代完整 S 主路线、L、第三方对照或重复缓存的生产故障测试。

### L 原生 rolling GPU 接入

新增生产 `RollingMotion` 条件预处理及 `tools/road_estimator.sh`。无噪声 80 km/h+yaw+非单位外参、恒加速度及缺 IMU 覆盖测试通过；保留全部原始样本字段，未加载真值用于去畸变。

固定输入为原 L 冻结包。命令模板：

```bash
data-run -- tools/road_estimator.sh --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 --config packages/liuo_tunnel/config/road_rolling_lio.json --seconds 120
```

- `20261027T154331Z_498`：20 s 首次运行失败。独立逐字节语义复查证明 LiDAR/UWB 完整，但 observer 漏收最早 3 条 IMU 和 3 条 clock，起始 IMU 从 1.015 s 开始。没有放宽计数/hash 条件；修复 DDS 图发现早于实际读者匹配的启动竞态，发布前等待 1 s、不发布预热传感器。
- `20261027T154547Z_447`：修复后同一 20 s 片段 PASS。200 cloud / 4001 IMU / 840 UWB batch / 4841 clock 全部 count/hash 一致；输出 179 个状态，t=3.1–20.9，实际发布 20.00153 s。
- `20261027T154655Z_668`：120 s 片段 PASS。1200 cloud / 24001 IMU / 6786 UWB batch / 30787 clock 全部核对；1179 输出覆盖 t=3.1–120.9，实际发布 120.00172 s。独立真值仅用于范围检查：到达 s≈1809.38 m，包含第一段左弯，最高 **70 km/h**，不能作为 80 km/h 生产证据。
- `20261027T155126Z_709`：180 s 片段失败。节点在 measurement_stamp=175.2 报 `OPTIMIZATION_FAILED`，GTSAM Cholesky 在 x1712 附近检测到欠约束系统。80 km/h 从场景 t≈161.315 开始，失败之前已进入该区间，但 180 s 未完成，不能判 PASS。保留失败和原始噪声模型。下一项为同一因子/输入仅改 QR 的数值求解诊断，结果未预判。

这些入口检查均为 GPU、1×、原始 rolling LIO-only；没有完成 L 未知站点自标定，也未证明定位有效。代码、配置与小闭环验收不能替代 C01–C16 的完整矩阵。

### L 起步发散诊断（预声明）

进一步检查失败 run `20261027T155126Z_709`：t=20 时 vx=-4.70058 m/s，t=175 时 vz=-58598.84 m/s。问题远早于最终 Cholesky 异常，不能只归因为求解器数值失败。t=15→20 的估计加计 bias x 从 0.0294 增至 0.9399 m/s²；输入合同 PASS 不证明这些轨迹有效。

已有 S 生产配置使用观测法向弱平移投影，初始 L 配置继承 phase1 默认而未开启。下一项先用 `road_rolling_lio_projected.json` 进行 30 s 起步对照；与原始 L 配置逐键仅 `project_weak_lidar_translation=true` 不同。保留已有法向谱最大值 1% 阈值、原始 GPU/IMU 因子、噪声和关键帧数量，不加载几何或位置真值。检查逐 topic 原始输入一致性以及 t=10/20/30 位置、速度、bias 和投影秩；这是归因诊断，没有新增精度通过阈值。QR 诊断暂缓，因为首先要解释早期反向漂移。

投影 30 s 诊断 `20261027T160450Z_802` 完成：300 cloud / 6001 IMU / 1337 UWB / 7338 clock 原始 count/hash 一致，279 输出 t=3.1–30.9，实际 30.00113 s。t=20 的 vx=6.73625 m/s、ba_x=-0.00387 m/s²，冻结真值仅离线比较的同一时刻实际速度为 6.45246 m/s。这是起步改善，不是精度验收。

随后的 180 s 投影诊断 `20261027T160542Z_501` 在约 t=47 已出现大幅位置跳变（t=46 x=424.20、t=47 x=141.74 m），投影秩仍为 2，不能把投影开关视作修复。下一项 `road_rolling_lio_projected_dogleg.json` 相对投影配置仅加入既有 `use_dogleg=true`；同一输入、噪声、因子和关键帧数量，检查非线性步长控制是否能抑制该跳变。该诊断不增加位姿先验。

高速 TA 独立解析检查通过：半径 2998.125 m、最大 80 km/h、制动/匀速/加速，非零杆臂及旋转规范下，0.1 s 插值的点和距离误差满足预声明 0.00190 m 上界；另补齐全覆盖 IMU 中重复/逆序拒绝检查，rolling 共 4 测试通过。这些定向测试不验证整体定位效果。

180 s 投影诊断最终在 t=171.5、x1675 附近发生 `OPTIMIZATION_FAILED`，run `20261027T160542Z_501` 为 failed，未出具片段 PASS。

外部 Adapter 新增显式 rolling 模式：校验字段/存储/时间/ring 后返回原消息，snapshot 继续原零时间转换。新增测试先因缺 scan_model 参数失败，再实现并与既有 snapshot 测试一起通过（2 测试）。尚未改变外部 runner 的场景选择、配置、时长或输出路径；不能称完整 FAST-LIO2 接通。

Dogleg 诊断 `20261027T160857Z_694`：t=47 x=307.52 m，t=60 x=-599.96 m，仍发生早期跳变；最终 t=122.545 因 cloud_queue_limit 退出。该次中途并行执行了独立 FAST-LIO2 构建，不作为实时资源验收。没有证据支持仅更换求解器即可修复。

下一项 `road_rolling_lio_bounded.json` 与 Dogleg 配置只修改关键帧连接预算：connection_window=1、max_keyframes=1，与既有 S 自标定生产设置一致。旧 L 配置一窗口最终约 1596 因子；本对照检查长期历史关键帧连接的影响，不改变噪声/物理输入/真值边界。新运行与构建串行，仍以全输入和 1×为要求，未预判精度。

FAST-LIO2 原生 20 s 接入 run `20261027T161207Z_043`，提交 `619f462`：200 cloud、4001 IMU、4841 clock，196 输出；实际发布 20.00164 s，原始/适配后/独立订阅者 LiDAR 和 IMU 语义 hash 全相同，原生进程退出 0。未发布 UWB/真值/LIUO 地图。仅完成接入冒烟，不是完整第三方性能对照；上游启动跳过初始点云的日志原样保留。

有界关键帧 180 s run `20261027T161319Z_773` 完成输入合同：1800 cloud / 36001 IMU / 10216 UWB / 46217 clock 全部 count/hash 一致，1779 输出覆盖 t=3.1–180.9，发布 180.00175 s。固定首输出 yaw/平移规范（不拟合全轨迹）的离线位置 RMSE **1169.14 m**，最大误差 2956.52 m，1 s RPE 平移 RMSE 32.28 m，速度 RMSE 20.22 m/s。该轨迹严重失效；完整输入与持续输出不代表定位有效。

新增 `tools/analyze_road_diagnostic.py` 只在离线端读取真值，写入各 run 的 `export/analysis/road_fixed_start_diagnostic.{json,png}`。其单元检查注入二次增长漂移，证明首时刻规范变换不会把后续误差拟合掉。此辅助规范仅移除初始 yaw/平移，保留 roll/pitch 差异；不作为 FAST-LIO 局部重力规范的最终跨方法主评估，也不应用于地图复用的坐标对齐。20 s FAST-LIO 和 30 s LIUO 诊断时长不同，不排名。

下一项通用候选生产模式 `self_calibrate_free` 已实现、正在定向验证。它拒绝全局地图/安装输入，车辆图持续边缘化，当前窗口联合候选不把旧位姿变成观测。现有 7 个候选和真实静态图生命周期测试通过；不能据此宣称生产未知 A 或 A-only 地图成功。

生产验证预声明：新 `road_free_anchor_diagnostic.json` 用相同冻结 L 的前 120 s，涵盖起步和首次左弯；记录候选事件、实际 A 数量、过期 TA、输入完整性和输出覆盖，不以纯 LIO 或全 pending 作为 C03 通过。旧 snapshot 使用新二进制做 8 s 独立冒烟，确认未改变旧模式入口；不作为完整 S 证据。

### 新普通 S 输入的交付门限（实施中）

已复现旧 S 缺少 reporter→车辆门限。新增显式 `uwb.enforce_reporter_delivery=true`（旧配置默认 false，历史包不修改）：AA 先完成原有两站 range/LOS 物理测距，推进独立样本编号与物理 deadline，再单独检查 reporter→tag 的 800 m 三维 range 和全世界 LOS，失败不发布。物理样本无论交付成功均记录独立 `physical_uwb.csv`；既有 `uwb.csv` 只记录交付，静态派生器仍不得读取全局物理日志。保留 250/s 调度，不因不可交付而减少物理测距或增加基站。此 S 软件交付为零额外延迟的理想模型，非射频/MAC 或 L 有限延迟模型。

新 `straight_reachable_capture.json` 只开启该生成端门限，首项 8 s 检查要求有可达与不可交付的独立 AA、物理样本数=交付+阻断、逐秒总测距≤250、传感器包中无被阻断样本。旧 snapshot 二进制回归单独保留原配置。正式 540 s 新输入尚未生成。

新二进制首个 snapshot 8 s 回归 `20261027T162550Z_889` 在模拟器启动即退出 127：安装环境缺少 `libliuo_tunnel_renderer.so` 的库路径导出，未开始生成。库已在数据盘安装，并非物理/算法失败。补齐模拟器包的 ament 库导出后重新检查安装环境；不使用系统盘 fallback。

通用候选生产 run `20261027T162236Z_439`（560fb26）完成 120 s 输入合同，1179 输出、最终 101 个车辆状态，无地图/安装因子。**实际 A=0，不能通过 C03。** 候选事件为 99 次球面不一致、24 次未观测圆周、48 次基线不足；两次真正联合候选各优化两个分支，最终分离 107.01/276.76 m，明确拒绝提交，未选取一个分支冒充唯一解。记录中的最大 pending=801。该失败触发 C04 局部几何需求评估；不能用当前切线约束远站，也不能以增加全局轴绕过。

旧 snapshot 8 s 回归 `20261027T162639Z_700`（3d65368）单 run PASS，1600 IMU / 80 cloud / 510 UWB / 1600 clock；59 个状态、初始输出 3.1 s，实际 GPU，处理 p95 7.82 ms。仅回归入口，不是完整 S 验收。

独立新 S 交付检查 `20261027T162701Z_360`（3d65368）是 capture-only：510 次物理测距、其中 48 次 AA，24 次交付、24 次报告者不可达；包内实际 486 条 UWB。独立解析矩形 LOS 与真值 Hermite 插值核查报告距离，误差最大 1.21e-8 m（预声明 0.002 m），逐秒物理交换最大 66≤250，包内身份集合严格等于预期交付且无阻断样本。与上一个相同 seed/时长、未开启交付门限的回归逐样本比较，物理时间/端点/噪声样本/值完全相同，证实仅改变交付，没有少做物理测距。报告为 `export/analysis/straight_reporter_delivery.json`。

```bash
data-run -- tools/run.sh --formal --capture-only --duration 8 --config packages/liuo_tunnel/config/straight_reachable_capture.json --wall-timeout 90
# 在 tools/environment.sh 同等 ROS/消息环境下执行：
data-run -- python3 tools/check_straight_reporter_delivery.py /home/dev/ws_iap/.runtime/0/liuo/20261027T162701Z_360 --reference /home/dev/ws_iap/.runtime/0/liuo/20261027T162639Z_700
```

### 有限局部墙面生产诊断（实施中）

基于上一项全 pending 的生产证据，新增不含全局 L 的 `LocalInstallationFactor(X,A)` 和有限观测域，合同见 `docs/spec/curved_tunnel_estimator_contract.md` C04 节。四个定向测试通过：body 墙面在旋转坐标下的中心/符号及范围检查、无半径输入的缓弯/强弯差异、右扰动/Anchor 导数有限差分（1e-7）、synthetic 元数据字段与来源拒绝。开发时先因缺模块失败，随后修复 GTSAM OptionalMatrixType 及显式 Point3 重载编译问题；未改变几何或数值判据。

首个生产诊断预声明为 L 冻结包前 60 s，GPU、1×、原噪声，仅新增实际点云的有限局部约束和既有 synthetic 偏距；不读 Anchor 坐标/高度。观测与候选日志必须显示实际支持、分支及是否加入 A；A=0 仍为初始化未完成。该步骤不是完整 C04/C14 验收，也未实现 A-only 地图往返。

首轮局部墙面 run `20261027T165014Z_522`（770db15）完成 60 s 输入合同：600 cloud、12001 IMU、3407 UWB、15408 clock，579 输出，发布 60.00144 s。**A=0，初始化未完成**；五次未解决分支，包含 id 3 的约 3.72 m 高度分离。墙面宽度约 11.197 m、中心约 1.875 m，但混合两侧的高度极值关联未提供足够区分。不能以输入 PASS 宣称 C04 通过。

新增逐侧实测地面关联，预声明见合同；五项定向测试通过，新增独立水平地面解析用例检查地面距离、地下候选拒绝及外推余量增长。接下来在相同冻结 L 前 60 s 检查实际支持、分支及 A，仍非完整路线验收。

逐侧地面 run `20261027T170426Z_913`（07cc7ab）仍 **A=0**。输入/输出与前一项同计数，1×发布 60.00180 s；58 次观测尝试有 39 次有效、9 次速度待定、5 次墙面不连续、3 次地面支持不足、2 次地面未观测。六个地下候选被明确拒绝，未解决分支降为两次；id 3 的两个相继窗口都剩有效上方分支，但未满足原 0.5 m 跨窗口位置一致性，不能提交。新增完整候选 XY 和稳定性差值日志以定位轴向或横向漂移，不修改原判据。地面关联提供了实际新信息，但没有解决初始化整体失败。

日志定位 run `20261027T170733Z_055`（f5c49af）重复同一 60 s 原始输入，600/12001/3407/15408 四类 count/hash 全匹配，579 输出；未改变候选判据。id 3 两个有效候选约为 `(496.449,-2.391)` 与 `(495.673,-2.182)` m，三维变化 **0.806112 m**，以纵向变化为主，超过原 0.5 m 检查，因此仍 A=0。该结果定位了拒绝来源，不能作为初始化通过。后续须区分同一公共车辆参考下的候选相对变化与联合图坐标变化，保持旧结果，不以放宽距离门限宣称修复。

公共车辆参考日志 run `20261027T171117Z_273`（51842ac）：原 60 s 输入合同 PASS，仍 A=0；id 3 世界变化 2.206437/2.268241 m，与同一 X490 的相对变化 0.005265/0.086237 m 明显不同，支持坐标比较缺陷诊断。后续局部候选改用公共车辆参考，门限不变；旧结果不提升为通过。该检查不代表绝对地图位置稳定。

公共参考修订的下一生产诊断预声明为同一冻结 L 前 120 s，覆盖首次左弯；保持原 GPU、1×和资源限制。检查实际 A 入图、原始 TA 移交、边缘化后持久状态及失败日志；存在 A 也不自动通过定位、地图复用或 C14。世界与相对一致性两种差值均保留，不能借坐标修订隐藏绝对地图误差。

公共参考版本 f986854 的 120 s 计划 run `20261027T171501Z_157` 在 t≈56.2 s 失败，未完成输入合同。id 3 相对变化 0.005289 m 已通过一致性，但直接把候选图世界 A 初值配上主图 X，导致 `LOCAL_INSTALLATION_OUTSIDE_OBSERVED_PATCH`。这是首次实际移交触发的坐标不一致，不能归为测量支持失败或初始化完成。修订为经相同局部观测 key 转换 A 初值，保留所有原始图信息及支持门限。七项测试已在前一版本通过；新增移交坐标解析回归。

坐标移交修复后的 run `20261027T171841Z_743`（c70ca2c）完成 120 s：1200 cloud / 24001 IMU / 6786 UWB / 30787 clock 的发布与独立 observer count/hash 全匹配，1179 输出覆盖 3.1–120.9 s，1×发布 120.00160 s。id 3 在 56.1 s **实际联合入图**，一次移交 200 条当前窗口原始 TA、0 AA；A=1 保持至末端，车辆 X/V/B 各最多 101。初值坐标转换距离 15.354 m 明确记录，不将这次转换或候选位姿作为观测。新增坐标解析回归与既有局部几何共 8 项测试通过；开发时测试 Eigen 表达式触发 Point3/Matrix 重载歧义，显式 Point3 后修复，未改数学判据。

离线 `tools/analyze_road_diagnostic.py` 固定首输出 yaw/平移、无全路线拟合：位置 RMSE 7.8336 m（XYZ 4.0791/6.6576/0.6352 m），最大 16.9861 m，姿态 RMSE 0.5331°，速度 RMSE 0.3199 m/s；1 s RPE 平移 RMSE 1.4508 m。文件位于 `export/analysis/road_fixed_start_diagnostic.{json,png}`。这只是局部核心首次取得非零 A 的开发证据，不是定位有效、地图可复用、静态 AA 或 C14 完成。

118 次局部观测有 57 次有效、38 次地面支持/平面性拒绝、9 次地面未观测、9 次速度待定、5 次墙面不连续。后半段持续地面支持拒绝，阻止后续 Anchor 关联；另有 5 次未解决位置分支和 1 次候选越界求解失败均保留。下一步须拆解实际地面支持失败的具体量，不能整体放宽判据。A-only 相关地图导出与独立复用、L 静态交付及完整 S/L 矩阵仍未完成。

```bash
data-run -- tools/road_estimator.sh --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 --config packages/liuo_tunnel/config/road_local_anchor_diagnostic.json --installation /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968/metadata/config/installation_metadata.json --seconds 120
data-run -- python3 tools/analyze_road_diagnostic.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T171841Z_743 --truth-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968
```

地面判据日志 run `20261027T172502Z_879`（4b09cb3）完成同一 120 s 输入合同：1179 输出，A3 于 56.2 s 实际入图。后半段失败可归因于固定纵向端点要求：例如 t=115 s 内点184、λ_min=0.000195 m²、λ_mid=0.060364 m²、横向0.831915 m，实际纵向 −9.193472…12.162657 m，只有 −10 m 端点不满足。修订为每侧地面实测域内关联，保留所有残差/散布/数量判据和墙面域，新增短地面域的正反例解析测试；下一步仍用同一 120 s 输入验证，不通过改原输入掩盖问题。

实测支持区间 run `20261027T172902Z_384`（677bcbf）完成 120 s 输入合同，1179 输出。id 3 于56.2 s、id 7 于100.6 s 真正入图（后者处于首次左弯段），分别移交200/201条当前窗口原始TA；两座持久A保留至结束，其他高度分支未强选。离线固定首时刻规范位置RMSE 7.8717 m、最大16.5682 m、速度RMSE 0.3055 m/s；不宣称定位精度通过。9项局部/坐标测试通过，数据与诊断图保留于该run。

A-only 地图格式、完整交叉协方差先验、原子往返与精确静态账本共6项地图测试通过（含旧v1/v2回归）。下一生产诊断预声明为相同冻结L前180 s，覆盖80 km/h段，检查真实A集合导出、3N维协方差与未初始化身份；尚未接通独立地图复用，不以JSON成功或部分路线代替C09/C14。
