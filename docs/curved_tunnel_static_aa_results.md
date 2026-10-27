# 曲线与静态 AA 开发记录

状态：**开发中，C01–C16 尚未完成，不是算法验收报告**。最近完整S运行200427已完成全部输入/输出，但定位RMSE1819.51m且无AA融合，仍属失败；详见文末流形先验修复后的完整S结果。

### 首次完整 L 独立复用：有实际接入，定位仍退化

run `20261027T184325Z_343`，启动提交 `cc5bca2`，使用独立 seed20261028 的完整输入 `20261027T182209Z_414`，加载完整 CAL `20261027T183140Z_829/export/map/calibrated_map.json`（SHA256 `8db1145e33fa3c5c48bb586d2837a425187901e149b8fa7de0a02be380744823`）。入口 `tools/road_estimator.sh` 使用 `road_anchor_map_reuse.json`、新输入 installation_metadata.json 和 `--seconds 392.6`，经 data-run 执行。

`road_reuse_seam_audit.json` 证明 140.4 秒加入一次旧地图先验，最终9个A[2,3,5,7,12,13,14,16,18]；730个新AA实际交付并进入回调，369叶/5边融合、361pending。旧图94叶与本轮身份分离。该旧版本没有复用后验导出，检查结果只为输入/图数量通过，不能推称精确后验叶身份通过；wire frame也没有独立采集字段，报告明确标false。

`road_fixed_map_diagnostic.json` 使用继承CAL坐标，未对齐REUSE起点：3905条输出中1373条接入前局部输出单列；140.4–393.5秒2532条地图输出位置RMSE146.3324m，XYZ63.9271/131.5764/3.76117m，最大256.3020m，姿态3.24410deg、速度1.58805m/s，1s/10s相对平移9.12232/34.26199m。此结果不构成完整L定位有效性验收，更不能因小于失败CAL误差而宣称成功。

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

A-only 真实导出 run `20261027T173616Z_391`（781b4f1）完成180 s输入合同：1800 cloud / 36001 IMU / 10216 UWB / 46217 clock全计数/语义hash一致，1779输出覆盖3.1–180.9 s，1×发布180.00159 s。id 3/7/14/5分别在56.4/100.2/140.6/178.8 s实际入图，车辆状态最多101。地图 `export/map/calibrated_map.json` 的检查点为180.0 s，四座A、14个未初始化身份、12×12完整协方差；最小特征值0.000109815、最大32.9151，跨Anchor协方差块范数4.89892。不存在reference字段，配置hash匹配、来源为干净781b4f13b1b87eb9712aea20fa55464aa4e706dd；地图SHA256 `b5c9b65dd8720b5c49f85018db3d7412b7c5368a5c27b7a6309e1209287b45af`。独立检查记录 `export/analysis/anchor_map_audit.json`。

此图明确 `AA_DISABLED` 和 `INITIALIZER_NOT_IMPLEMENTED`；是实际联合后验导出，不是已验证的复用地图。离线固定首输出规范的位置RMSE17.7928 m，XYZ为6.7199/16.4654/0.5642 m，最大41.2978 m、姿态RMSE0.8098°、速度RMSE0.3537 m/s；不能以对照失效或协方差正定宣布有效/一致。诊断JSON/图保留在run内。

只从该run已交付UWB记录读取到AA端点：2–13、3–16、5–18、9–5、11–3、16–7、18–2；每条均至少有一个尚未初始化端点，因此这轮TA-only未产生实际AA入图。后续必须接通合法交付的静态记录与有界联合候选，保持未知端点不凭空创建、同一物理块仅计权一次；不从全局物理日志补图。新地图的生产复用初始化、独立第二轮和完整S/L矩阵仍未完成。

静态候选通路已新增只读pending摘要和同次主图事务移交；真实ISAM2两项生命周期/候选测试通过。S与L派生器两项Python测试通过，L检查正反交付及driver重启保持物理身份、新物理序号才增加样本、缺freshness拒绝。生产验证预声明为冻结L前180 s，新 `road_local_static_diagnostic.json`，与此前TA-only的原始LiDAR/IMU/UWB/clock完全相同；只从已交付AA生成项目本地静态记录。检查实际候选使用、静态物理叶集合、A和真实AA入图；仍全pending或AA=0时不得宣称静态主线完成。

静态联合候选 run `20261027T174621Z_416`（fbb273e）完成180 s输入合同，1779输出。所有四类原始输入计数/hash与前一TA-only 180 s相同，1×发布180.00103 s。A3在56.1 s、A16在79.4 s、A14在140.3 s、A5在178.8 s入图；A16候选实际使用连接A3的静态似然，同一次主图update移交79个独立物理AA，形成一条累计边。最终94个独立AA已入图、255个因端点未知待定；该边从79.4持续到180.9 s，超过车辆10 s lag。最早已吸收样本采集于1.012 s，未改成当前车辆时刻。A7等剩余多分支明确拒绝，不能把此轮当全Anchor初始化。

独立 `tools/check_road_static.sh` 重新读取冻结传感器包（仅已交付AA），逐条核对原始物理序号/无向端点/距离/采集时间与生产回调CDR，再核对地图账本。349条原始交付=349条接收=349个唯一物理叶；94个地图吸收叶逐值匹配，94+255=349。地图为180.0 s检查点，A=[3,16,14,5]、12×12完整相关协方差，最小特征值1.63254e-5；EXACT_LEDGER含94叶。记录为 `export/analysis/road_static_audit.json`，不读取全局physical日志。

同一固定首时刻规范离线位置RMSE18.8291 m、XYZ7.1084/17.4157/0.8360 m、最大44.1714 m，姿态RMSE0.8777°、速度RMSE0.4492 m/s。比先前TA-only 17.7928 m并未改善，因此本轮只证明持久静态AA和实际相关地图账本通路；不宣称精度收益、统计一致性或完整C14完成。新地图复用初始化与完整S/L矩阵仍须实现/执行。

```bash
data-run -- tools/road_estimator.sh --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 --config packages/liuo_tunnel/config/road_local_static_diagnostic.json --installation /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968/metadata/config/installation_metadata.json --seconds 180
data-run -- tools/check_road_static.sh /home/dev/ws_iap/.runtime/0/liuo/20261027T174621Z_416
```

地图复用基础模块推进：新增未知重力坐标关系G及单一完整相关MapFramePrior，没有车辆起点/朝向观测或旧bias。地图测试10项通过（含联合代价/Hessian、导数、不同起点的高斯优化测试模型和输出交叉协方差）；静态图测试5项通过（含延迟prior准备、旧/新信息分离、1000次重发、已融合重叠和内容冲突拒绝）。这些尚未被生产复用入口调用；旧入口仍拒绝A-only地图。下一项必须接通有界初始化、地图/本轮账本原子协调和带G交叉项的输出，再生成独立噪声数据验证，不能拿这些单元测试代替C09/C14。


### A-only 独立复用生产接入开发

基础模块提交669ba7c的10项地图/5项静态图测试通过。随后新增reuse_anchor_map节点入口、延迟联合G/A先验与旧AA隔离、全联合G/X/V输出协方差、reset后的新叶恢复、显式--map快照及同训练包拒绝。旧reuse_map仍保留；新模式在地图坐标关系建立前输出INITIALIZING/local odom，接入后仅DEGRADED/provisional，不称全局初始化认证。该记录是实现进展，尚无独立第二轮生产验收；本轮未启动新的定位回放，完整S/L矩阵仍未完成。


本次针对性检查：`data-run -- ... cmake --build ... --target test_calibrated_map test_static_edge_graph test_estimator_status -j2` 后分别执行三个二进制，12+5+13=30项通过；Python回放入口使用数据盘PYTHONPYCACHEPREFIX编译通过。首次测试编译发现gtest模板逗号宏括号问题，已修复；一次未设置缓存目录的py_compile被data-run拒绝在源码树写缓存，随后显式设置数据盘路径，未绕过隔离。

按code-review技能做规则/需求两路只读审查，发现两个P1：接入地图后已有A的待定TA重复启动候选、复用状态覆盖LOST。均已修复并复核：真实待定TA与map prior同次加入，成功才消费队列并计入postfit；过期记录明确退休，未知A仍待定；状态保留原始故障与原因。新增对应回归。这里仍没有新的生产定位运行证据，不替代完整S/L验收。


### 独立L第二轮完整输入

生成提交89cb64e，run `/home/dev/ws_iap/.runtime/0/liuo/20261027T182209Z_414`，命令 `data-run -- tools/road_scene.sh --seed 20261028`。完整392.6秒、3926cloud/78521IMU/21860UWB/100381clock、77032491点；生成53.928秒（7.280×），含全包审计/图/接收检查的owner总耗时232.896秒，completed/pass仅指场景。原始SHA `d146412b1e3e7a93939ff0809c44dd1baf31184070f7fcccf5de5ab678fffcfa`，旧输入未改动。

独立比较器 `tools/check_road_independence.py --first <旧L> --second <新L>`（data-run及ROS/shared消息环境）通过：除seed配置相同，全部运动数组精确相同，站点/设施/安装资料相同；clock同摘要、三类传感器异摘要、数量/点数相同。IMU白噪声跨轮相关最大绝对值0.004717；21130条TA噪声相关−0.003943、4323条AA噪声相关−0.015134，均满足预声明0.05。两轮bias不同。报告在新run `export/analysis/independent_input.json`。首次比较器因反复读取NPZ整数组被主动中断，改为一次物化后通过；未改数据或容差。

固定CAL坐标评价加入 `--map-reference-run`，回归fixture保留预设15m起点误差，接入前样本单列。新reuse配置已绑定新L输入SHA，接下来的180秒运行仅检查新版生产复用接缝。


### 首次A-only独立生产复用180秒接缝检查

run `20261027T182811Z_609`，代码60f9f5a，GPU1×，输入为独立seed20261028完整L包的前180秒，加载 `20261027T174621Z_416` 的4A/94叶地图，快照SHA `900a9038250a2c8df46894127a1f7391761cc674e1d65922930ed308f67eea50`。命令为 `data-run -- tools/road_estimator.sh --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T182209Z_414 --config packages/liuo_tunnel/config/road_anchor_map_reuse.json --map /home/dev/ws_iap/.runtime/0/liuo/20261027T174621Z_416/export/map/calibrated_map.json --installation /home/dev/ws_iap/.runtime/0/liuo/20261027T182209Z_414/metadata/config/installation_metadata.json --seconds 180`。

输入检查PASS：1800cloud/36001IMU/10216UWB/46217clock，1779输出，墙时180.001137秒。A3@56.3、A7@99.3、A14@140.6由本轮原始图初始化；旧图不含A7，故至140.6才有两个公共A，真实加入一个联合MapFramePrior，旧图A16/A5经该prior进入图。全局G无独立先验，输出继承旧map_id，新增总A集合[3,5,7,14,16]，车辆101组仍有界。原始未初始化/LOST区间保留。

`tools/check_road_static.sh <run>`新增复用接缝审计：349原始独立AA=349回调，161新叶入图（3–16:94；7–16:67）、188pending、2显式边，旧图94叶与新身份不重叠；图中prior接入前0、之后1。此审计只证明交付与图计数，叶归属由已知端点和计数推断，当前复用入口没有直接导出最终精确后验叶账本，不声称已经完成该验收。结果 `road_reuse_seam_audit.json`。

固定旧CAL规范评价 `data-run -- python3 tools/analyze_road_diagnostic.py --run <reuse> --truth-run <newL> --map-reference-run <oldCAL>`：1375条接入前局部输出单列；140.6–180.9秒404条旧map输出RMSE32.1867m、XYZ10.9792/30.1533/2.49387m、最大41.7695m、姿态1.23838deg、速度0.45937m/s；1s/10s相对平移RMSE0.05600/0.38369m。未按第二轮起点或整条轨迹对齐。图/误差见 `road_fixed_map_diagnostic.{json,png}`。这证明生产接缝实际运行，不证明定位有效、任意启动或完整L复用完成；完整392.6秒CAL与之后全路REUSE继续。


### 首次完整L-CAL：输入/账本通过，后半程定位严重退化

run `20261027T183140Z_829`，启动提交60f9f5a，实际二进制与短复用相同。命令 `data-run -- tools/road_estimator.sh --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 --config packages/liuo_tunnel/config/road_local_static_diagnostic.json --installation /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968/metadata/config/installation_metadata.json --seconds 392.6`。全程3926cloud/78521IMU/21860UWB/100381clock严格核对，3905输出3.1–393.5秒，墙时392.601097秒；输入PASS不代表定位有效。

实际地图仍为4A[3,16,14,5]，初始化56.2/79.6/140.2/178.7秒，393.0秒完整后验已导出；730已交付独立AA=730回调，其中94叶/1边实际融合、636pending，精确地图账本审计PASS，协方差最小特征值1.64118e−5。该地图未覆盖所有站点，不把全路输入覆盖当成全路有效地图。

固定初始规范误差：全程位置RMSE1260.697m，XYZ1254.610/123.339/9.8968m，最大4213.836m，姿态8.63258deg、速度18.8649m/s，1s/10s相对平移23.5020/191.861m。DEGRADED2328条、LOST1577条，240.3秒后持续POSTERIOR_UNBOUNDED。车辆最大101组、因子754，RSS432MiB、整设备显存1192.31MiB，IMU/云/UWB最大队列421/0/9，最大优化51.507ms；未以扩大窗口或资源掩盖失败。报告 `road_fixed_start_diagnostic.json`、`road_static_audit.json`、`full_route_resources.json`。

初步根因证据在 `full_route_initialization_failure.json`：180–220秒40个局部几何片段中34个OBSERVED，故不能把最早失败简单归为无几何。A18@196.1仍有1.398m合法分支，A2@213.3/215.3有3.449/3.423m分支，随后A13@230–234有1.7–1.99m分支；220秒后floor观测失败明显增多。A2共同车辆参考中的两组z约−1.6405与+1.78237m，是真实相对高度分支，不是不同世界坐标造成的假分支；当前单片段可行区域/保守地面误差未排除下分支。之后UWB新站点不能进入图，LIO逐渐失控。下一步需核查多视角有限局部几何/有界分支信息保留如何增加合法辨识信息，不能缩小既有误差门限、强制固定高度或任意挑低代价解。全L功能验收未通过。


### 复用后验导出与审计修正（生产验证待做）

新增完整G/A联合边际传播到继承地图坐标、旧/新叶原子检查点、祖先原始输入SHA保留与独立输出坐标身份。审查发现全程eligible子集加数量不足以证明检查点身份；已改为该次graph_update之前已交付且端点就绪的精确集合，并要求唯一同stamp更新。新增未来同边叶等数量替换等四项负例/正例通过。

全部经data-run：节点构建通过；test_calibrated_map共13项通过，test_checkpoint_ledger_audit.py共4项通过，test_road_fixed_map_gauge.py共1项通过；五个相关Python入口语法检查通过。新增C++夹具先后修正Eigen表达式重载歧义和第三Anchor遗漏ID，未调整门限。旧完整REUSE仍无精确导出证据，新生产回放尚待完成。


### 精确复用后验生产检查180秒

run `20261027T185849Z_530`，启动提交1daa642，独立L输入182209，完整CAL图183140，其他参数同上，`--seconds 180`。GPU1×原始输入核对通过（1800cloud/36001IMU/10216UWB/46217clock），1779输出、墙时180.001224秒。140.4秒接入一次旧图，180.0秒后验实际导出：旧94叶精确保留，新161叶精确等于该次图更新前已交付且端点就绪集合，188叶pending，5个A，协方差最小特征值1.16605e−5。独立接收端frame_id确认odom→map，子frame为imu。继承地图SHA、全部祖先原始输入SHA和旧叶payload均核对通过；结果为PASS_INPUT_GRAPH_AND_EXACT_POSTERIOR_LEDGER，非定位验收。

命令 `data-run -- tools/check_road_static.sh /home/dev/ws_iap/.runtime/0/liuo/20261027T185849Z_530`；评价继续使用 `tools/analyze_road_diagnostic.py --map-reference-run <183140>`。1373条接入前输出单列，406条地图输出140.4–180.9秒，RMSE31.8545m、最大41.4026m、姿态1.229deg、速度0.45610m/s；1s/10s相对平移0.05462/0.39856m。定位问题仍在，不能用该机制检查代替完整S/L主线。


完整REUSE184325的资源附加核查：`data-run -- env PYTHONDONTWRITEBYTECODE=1 python3 tools/summarize_estimator_resources.py <run>`，报告 `full_route_resources.json`。3905输出中INITIALIZING1340、LOST1200、DEGRADED1365；车辆最多101组、因子862、RSS429.95MiB、整CUDA设备1191MiB、IMU/云/UWB队列421/0/8、优化最大71.2194ms、保留点云101帧/keyframe1。资源未无限增长不代表定位有效，也不替代规定的60秒故障/慢订阅实验。


### 完整普通S首轮输入与snapshot接入

首轮输入run `20261027T190229Z_933`，干净生成提交f58b400，seed0、起点[0,0,1.2,0]，capture-only540秒。5400cloud/108000IMU/39960UWB/108000clock，真实x=0→2350m，末真值stamp540.9，首尾静止；原始MCAP SHA256 `8de5439e8e35ae747ee42842aa505b8ec5290548a70b4386b47c71df82ccdd7a`。原有S包包含单独truth topic，回放存储过滤排除，不改写原包；此处不冒称S包是L那种纯四话题包。

`check_straight_inputs.py --first <run>` 全程/原始及接收摘要/IMU合成残差检查通过。`check_straight_reporter_delivery.py <run>`独立检查通过：41395物理交换，其中3240AA、1805AA实际交付、1435报告不可达；总39960条交付，最大84交换/s，报告者距离独立误差2.012e−8m。检查器在data-run及ROS/shared msg环境执行，没有向算法提供全局物理日志。此为新完整输入，旧360秒数据保持。第二轮独立输入仍需生成和交叉检查。

S接入失败191153：旧包额外truth topic被新严格白名单拒绝，输入发布前结束；修复为S特有已知truth话题的存储级排除，其他额外话题仍拒绝。失败191231：20秒包含边界201帧却没有其后续IMU，末帧消费检查失败；改为S片段半开区间，未放宽检查。成功run `20261027T191331Z_091`（2b6c560）：200cloud/4000IMU/1215UWB/4000clock逐字段hash一致，179输出3.1–20.9秒，GPU1×约19.9954秒。`sensor_access_boundary.json`明确排除真值。

此短段只证明新A-only入口的S原生消息接通，尚无Anchor初始化/地图。A0在洞口，最初运动时缺少两侧连续墙面，14秒起OBSERVED时已驶过其可关联范围；没有改变局部跨度或容差强行接入。完整路线须继续检查后续站点，不能将20秒接入PASS当成S-CAL完成。


### 完整普通S独立第二轮输入

run `20261027T191449Z_257`，生成提交42d7270，seed1、起点[0,0.2,1.6,pi/2]。540秒、x0→2350m，5400cloud/108000IMU/39827UWB/108000clock；原始MCAP SHA256 `1621fe02e414c14a788f5f1d8c73fb6d9779e4be41a48201a30f59c80e48e329`。与CAL传感器配置完全相同、相同实际生成二进制，场景只允许seed与起点/朝向不同。原始/接收文件摘要复核、clock相同hash和三类传感器不同hash均通过。

`data-run -- env PYTHONDONTWRITEBYTECODE=1 python3 tools/check_straight_inputs.py --first <190229> --second <191449>`通过：IMU扣除各自运动/偏置后的逐轴白噪声相关最大绝对值0.0066485；匹配37876个TA物理样本相关−0.00483268，3240个AA相关0.0299842，均满足预声明0.05。不同位姿导致实际可达/交付数量不同，不强行要求UWB条数相等。报告 `export/analysis/straight_inputs.json`。

独立报告者检查 `straight_reporter_delivery.json`通过：41268物理交换、3240AA，其中1799交付/1441被阻；最大84交换/s，报告者距离误差2.012e−8m。保留全部原始输入，未启动估计器生成该包。完整S-CAL已从首轮冻结输入启动为 `20261027T192427Z_824`（2c29438），540秒；本记录时仍在运行，不提前给算法结论。

静态交付最小策略新增真实ROS消息/CDR测试3项通过（1000重复、不跨报告者缓存、32块界限），精确账本回归4项仍通过。两种策略的实际生产STATIC-PAIR尚待运行；实现和预声明在static_anchor_edges_contract.md，不冒称消息单测已完成信息/通信生产验收。


### 首次完整普通S-CAL失败及批内坐标判据诊断

run `20261027T192427Z_824`（2c29438）完整540秒结束，GPU1×墙时539.995924秒，5400cloud/108000IMU/39960UWB/108000clock严格核对，5379输出3.1–540.9秒。没有Anchor初始化，没有地图，1805条真实已交付AA全部进入回调但仍pending、0融合。`check_road_static.sh`现在对这种情况保存INCOMPLETE_NO_EXPORTED_MAP并返回非零，绝不把输入通过转成算法通过。

固定首输出yaw/translation规范位置RMSE8144.878m、XYZ7319.834/2400.738/2644.906m、最大37465.521m；姿态22.3747deg、速度158.1295m/s，1s/10s相对平移190.539/1750.962m。DEGRADED442、LOST4937；最多101组车辆、398因子、RSS336.01MiB、整设备1177.125MiB、IMU/云/UWB队列402/0/2、优化最大33.7761ms。资源有界但算法严重发散，不能作为有效S主线。

独立记录解析 `tools/analyze_candidate_branches.py`发现A1在95.4–97.8秒的四个有限候选全以X880为共同参考：相对最佳解最大差0.052764m，世界坐标差41.369225m。旧批内世界距离门限错误地把共同局部几何计为分支；A2另一个批次仍有真实相对分支。旧完整L-CAL的13个拒绝批次全部仍有相对分支，不能把它们归为此缺陷。报告为candidate_branch_diagnostic.json和relative_branch_gate_diagnostic.json。

修复局部批内检查采用现有共同车辆参考，0.5m门限不变，没有修改地面误差或安装高度。11项local_installation/relative_candidate测试已通过，包含S实测四候选回归及L-A2真实3.4m高度分支必须保留的反例。节点构建已通过，新生产验证继续；当前失败仍绑定旧二进制，不会被新代码追认。


### S共同参考分支修复后的110秒目标运行提前失败

`20261027T193517Z_889`（a55a4c9）目标110秒，但98.7秒以192个当前原始TA实际初始化A1后，98.905秒发生OPTIMIZATION_FAILED/LOCAL_INSTALLATION_OUTSIDE_OBSERVED_PATCH并停止。仅有一个Anchor的临时检查点保留，不能用于要求至少两点的地图坐标复用。该次候选世界距离本来也小于0.5m；因此初始化本身不能证明消除旧41m分支误判的因果效果，直接修复依据是旧记录/回归夹具。

后续有限域处理将域外非线性试探代价设为无限，让原Dogleg/LM拒绝试探，而不在域外外推墙面。无效线性化仍抛错，导出/发布前检查仍显式的局部因子。12项局部几何/共同参考测试通过，新增真实GTSAM Dogleg测试确认有域外试探、最终仍在±10m内且有限代价下降。没有增加跨度、放松误差或增加高度先验。同输入生产重放结果见下，失败不改写为通过。


### 有限域试探保护后的生产失败与诊断范围

`20261027T194432Z_323`绑定681e4f9，目标110秒。99.3秒以192个原始TA初始化A1，99.505秒再次失败；错误明确为`linearization=1;along=495.749259`，不是域外试探代价。因而试探保护不足以解决生产问题。候选共同参考仍为X880，批内相对距离至多0.130682m，而世界距离最大206.716476m；此次确有世界/共同参考判据差异，但完整定位与地图复用仍未通过。

下一步只增加优化器失败状态记录：最多512个变量，保存线性化点、现有增量及固定线性化标志；记录器不调用calculateEstimate，不再次优化、不回退、不改变判据。将核对旧车辆线性化点与新Anchor线性化点是否不一致。当前这只是待验证原因；IMU采样噪声到连续密度的除以sqrt(rate)实现已核查，未发现该处错误。


诊断运行`20261027T195239Z_132`（ba22f75）在99.8秒以193个TA初始化A1，100.005秒因局部因子线性化along=−28.267005失败。`profiling/optimizer_failure_state.json`保存全部307个变量，未截断。A1的线性化点已固定，X880未固定；A1现有增量为[−28.842539,0.110303,0.102708]m。独立SciPy旋转变换得到X880下原始theta的A1为[28.327740,2.973264,3.834092]m，而仅对固定A1加回其已有delta后为[−0.513778,2.733813,3.989781]m。见`export/analysis/finite_patch_chart_diagnostic.json`。这支持固定A与可重线性化X的坐标参考不一致方向，但不是重建完整已接受状态，也未证明X880就是失败因子的关联车辆；下一步需带实际因子关联做最小优化器回归。不得直接扩大补丁范围、删掉已吸收信息或把该报告称为定位通过。


### 固定线性化参考的最小复现与流形边缘先验修复

新增`test_marginal_chart`用实际项目ISAM2、旧车辆边缘化、持久Anchor和实际有限LocalInstallationFactor复现：测试中的车辆/Anchor共同移动30m，旧实现仍抛出`linearization=1;along=-29.999970`。测试中的笛卡尔相对观测和位姿先验仅用于构造可解反例，不进入生产估计器。旧失败日志`/home/dev/ws_iap/liuo_observation/log/marginal_chart_before.log`。

修复保留每个边缘先验的原始二次型/参考，使用完整Local流形Jacobian表达于当前求解坐标，不再固定活动A/X的theta。共同平移复现测试、非零旋转/大平移的独立有限差分梯度及交叉信息测试、原联合Schur信息/代价回归共3项通过，日志`marginal_chart_after.log`。测试显式将构建目录置于LD_LIBRARY_PATH最前，避免误测旧已安装库。生产完整S/L尚未验证该版本，阶段仍未完成。

生产节点构建/安装通过；上述3项加局部几何12项、静态图5项、地图13项，共33项通过。汇总日志`/home/dev/ws_iap/liuo_observation/log/manifold_marginal_tests.log`。


### 流形边缘先验修复后的S 110秒重放

`20261027T200157Z_906`绑定1ff22c1，同一冻结S-CAL输入，GPU、1×、110秒完成：1100云/22000IMU/7080UWB/22000clock，1079个输出覆盖3.1–110.9秒，发布墙钟109.995690秒。A1在99.4秒以192个原始TA初始化，越过原约100秒失败点，未再抛出有限补丁线性化异常。

这是异常修复证据，不是定位通过。固定起点yaw/translation坐标下RMSE36.203743m（XYZ36.190785/0.384804/0.888859），最大119.437560m，姿态0.293457deg，速度1.338995m/s；RPE1s13.115417m/10s36.068308m。DEGRADED556/LOST523。110.0秒地图只有A1，最小协方差特征值0.0390806；350个AA全部pending，0融合，因此不能作非平凡地图复用。原始AA身份与回调/账本审计PASS仅覆盖交付一致性。

资源最大101车辆状态、586因子、RSS324.281MiB、整卡1159.25MiB，IMU/cloud/UWB队列402/0/2，最大优化67.729ms。分析文件snapshot_smoke.json、road_static_audit.json、road_fixed_start_diagnostic.json/png和full_route_resources.json均保留。下一步同设置完整540秒S路线检查后续Anchor和实际AA。


### 资源时间趋势与第三方输入边界补充

资源检查器新增按session分别统计的首末值、变化量、最小二乘变化率及四个等时长窗口的样本数/中位数/最大值，覆盖RSS、整卡显存、车辆状态、因子、Anchor和优化时间。已用独立线性序列、不规则采样、session重置和逆序拒绝用例检查，并对S110运行200157生成实际报告。趋势是描述性证据，不自动判定无泄漏，也不代替60秒生产负载。

FAST-LIO外部入口在首条read_next之前设置仅LiDAR/IMU/clock的StorageFilter，静止启动单独仅选择IMU；legacy S真值/UWB原始payload不进入读取循环。snapshot时段为左闭右开，与原生LIUO一致；rolling保留既有包含末时刻合同。选择/排除topic及端点语义记录在baseline_input manifest。3项适配/过滤顺序测试通过（baseline_input_boundary_tests.log）；真实冻结S-CAL前20秒的过滤后4000IMU/200云计数和原始语义hash，与生产191331输出完全相等（baseline_s20_filtered_input.json）。上述日志均在/home/dev/ws_iap/liuo_observation/log。此次未启动第三方算法，完整对照仍待运行。


### 流形先验修复后完整S：异常消失，定位与AA主线仍失败

run `20261027T200427Z_951`，启动提交71e3075，实际算法改动1ff22c1，输入为190229，配置straight_local_static.json和straight_local_offsets.json，`data-run -- tools/road_estimator.sh --scene straight --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T190229Z_933 --config packages/liuo_tunnel/config/straight_local_static.json --installation packages/liuo_tunnel/config/straight_local_offsets.json --seconds 540`。5400云/108000IMU/39960UWB/108000clock严格核对，5379输出3.1–540.9秒，墙钟539.995781秒；无原有限补丁线性化异常。

A2在191.9秒、A4在364.4秒各以192原始TA初始化。540.0秒真实地图只有[2,4]，协方差最小特征值8.40132e−5；1805交付AA全部pending，0融合、0边。静态审计PASS仅证明交付身份/数量和空融合账本一致，不满足静态AA有效性。A1在94.7秒出现真实共同参考分支5.252725m，99.3秒只有首次稳定候选，未在窗口内再次确认；A3/A5多次不满足观测高度/后续无有效sphere seed。完整运行与S110首次A1成功不同，不能把单次短片段宣称为稳定初始化。

固定首输出yaw/translation规范RMSE1819.513912m（XYZ1818.778341/29.535221/42.472230），最大3433.275959m，姿态5.424448deg，速度8.676368m/s；RPE1s31.317288m、10s126.704384m。DEGRADED2316/LOST3063。仍为严重定位失败，不能因较旧8144.88m结果小而称有效。

资源最大101车辆状态、695因子、RSS335.164MiB、整卡1175.875MiB，IMU/云/UWB队列402/0/2，优化最大60.000ms。四等时窗RSS中位数298.184/316.645/327.977/330.961MiB，因子中位数380/567/569/568，车辆中位数均101；只作本次趋势描述，不声称长时无泄漏。原始包、两个Anchor地图、全部失败输出和分析图/JSON保留。后续需解决首个近距离候选的稳定确认与高度/几何关联，不放宽物理条件或几何判据。


### 流形先验修复后完整L：7站与真实AA，定位仍未通过

run `20261027T201351Z_374`，启动aeb042e，算法1ff22c1，与完整S实际节点和共享库相同。命令沿用road_local_static_diagnostic.json、冻结134947及其installation_metadata.json、`--seconds 392.6`。3926云/78521IMU/21860UWB/100381clock核对通过，3905输出覆盖3.1–393.5秒，墙钟392.601186秒。393.0秒地图包含[3,16,14,5,18,2,15]，240个AA融合到3边，490 pending，总交付730；首次融合80.3秒，包含采集于1.012秒的旧物理样本，超过车辆lag。最小地图协方差特征值1.241234e−5。

固定首输出规范RMSE115.486705m，XYZ54.151446/101.137057/13.270103m，最大341.463110m，姿态2.263290deg，速度1.392006m/s；RPE1s13.305710m/10s41.775770m。沿路线/左横向/世界高度分量为51.466381/102.529484/13.270103m。此为实际地图与AA证据，但定位仍未验收，不能仅以优于旧失败结果称有效。状态DEGRADED2551/LOST1354；最大101车辆状态、818因子、RSS420.750MiB、整卡1220.313MiB、IMU/云/UWB队列421/0/8、优化67.227ms。全部JSON、固定坐标误差图、路线分量图与时间趋势保留。

新增离线路线误差分解仅使用真值水平运动切向/左法向/世界竖直，不更改任何估计、规范或对齐；静止时采用时间最近的非零真值速度方向。独立坐标旋转/范数和既有两种格式15m复用偏差测试通过，日志`/home/dev/ws_iap/liuo_observation/log/route_frame_analysis_tests.log`。

S200427附加`bias_diagnostic.json`以独立合成噪声重建仅离线检查bias。预测纵向速度90秒0.5928m/s、110秒−1.7149m/s、191.8秒−8.0612m/s，192秒首次A2后−4.9722m/s。代码每次以estimated up×velocity选左法向，故存在速度反向导致固定安装侧别重新解释的风险；尚需最小测试与生产修复，不能直接用真值速度替换。


### 安装侧别不能随估计速度反向交换

S200427离线bias/速度记录显示首个Anchor前预测纵向速度已反向。旧观测函数每次取up×velocity选择正法向，固定synthetic偏距因而可能被解释到另一墙面。最小几何测试用完全相同的点云、姿态和偏距，仅将速度+5变为−5，旧选择确实翻转法向；修复采用最近成功局部观测的法向，经两个仍在联合窗口内X的相对旋转仅传递符号约定。当前速度零或反向不改变固定侧别。未改变墙面拟合、floor误差、±10m支持、候选分支门限或任何权重。

首次仍需有效运动建立侧别；参考过期/关联不明确时pending，不能把缓存位姿当已知量或读取道路真值补足。只保存一个key并在reset清除。新增local_orientation.csv记录参考key、速度及当前法向与motion-left的关系，供生产审计。节点构建/安装通过，14项局部几何/相对候选测试通过，日志local_orientation_build.log、local_orientation_tests.log、local_orientation_install.log均位于/home/dev/ws_iap/liuo_observation/log。生产验证仍待进行，不把历史失败追认为通过。


侧别修复生产运行`20261027T202541Z_963`（bbe9c5d）完成110秒，1100云/22000IMU/7080UWB/22000clock、1079输出，墙钟109.995496秒。local_orientation.csv有18个观测与当前motion-left反向，首个发生于93秒，仍传递原侧别；未因速度反向交换墙面。A1的89.5/95.4秒候选等待稳定，100.5秒有真实共同参考分支8.621910m，最终0A、无地图、350AA全pending。输入PASS不覆盖初始化；静态审计返回INCOMPLETE_NO_EXPORTED_MAP并exit1。固定坐标RMSE97.093279m，最大273.142308m，姿态0.368954deg、速度2.923048m/s；侧别语义修复成立，但不能声称已修复主定位问题。保留原门限，下一步执行独立输入的完整FAST-LIO2对照并继续定位候选/纯LIO失效原因。


### L地图逐站/站间与AA图的离线诊断

`data-run -- python3 tools/analyze_anchor_map.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T201351Z_374 --truth-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968`生成anchor_map_diagnostic.json/png。复用既有CAL固定yaw/translation，不拟合地图对齐；truth anchors.json及全局物理/交付日志只用于离线。7站位置RMSE90.719255m；A3/16/14/5/18/2/15误差分别3.386/5.263/22.297/40.738/51.975/65.335/220.100m。21个已初始化站对的最大距离误差50.286247m，距离方差使用完整Cii+Cjj−Cij−Cji，未丢弃相关项。部分位置误差明显超出报告标准差，不能宣称统计一致。

按全部18个注册站计，物理完整输入图和实际交付完整输入图均11边、7个连通分量，真值条件单位距离Jacobian秩11；检查点实际融合图3边、15个分量、秩3。移除公共刚体6维规范后的剩余零方向维数分别37/37/45。该秩只描述真值条件的距离几何，不是联合估计信息，也不要求AA子图独立刚性。最大联合地图协方差特征值403.381194m²，完整特征值及最大模式方向按Anchor顺序保留。

新增两项解析测试通过：公共平移相关项在站间距离中抵消且旋转等变；三角形加孤点正确报告连通分量、距离秩及规范。另新增独立FAST-LIO输入检查器，重新流式读取冻结包核对原始LiDAR/IMU语义摘要、适配订阅端摘要和1×时段；可用--liuo-run进一步核对配对。5个负例覆盖改原始样本、额外云、订阅丢IMU/改云及clock缺失。当前完整FAST-LIO仍在运行，尚无本次终态报告。


### 完整S独立输入的FAST-LIO2对照

`20261027T202832Z_457`绑定42fdde9，命令 `data-run -- tools/baselines/run.sh fastlio --source /home/dev/ws_iap/.runtime/0/liuo/20261027T191449Z_257 --scan-model snapshot --duration 540`。完整5400云/108000IMU/108000clock，5397输出1.3–540.899999999秒，发布墙钟539.995710秒，进程正常结束。独立重新读取冻结包的shared_input_audit.json通过，原始语义与适配订阅端数量/摘要完整；未提供配对LIUO运行，paired_liuo_verified=false，不能冒称完整OFF配对已完成。

固定首输出yaw/translation规范位置RMSE3277.206273m（XYZ3277.182477/8.059927/9.539821m），最大4913.915287m，姿态0.489650deg；1s/10s相对平移10.794105/108.820231m。没有速度字段，未编造速度指标。真实第三方定位失败和完整输入有效性分别记录，不据此声称LIUO有效。命令及图/JSON保留于该run的export/analysis。

新增straight_matched_lio_only.json和road_matched_lio_only.json，分别严格继承对应anchor_map_reuse配置的传感器和odometry；仅将UWB设record_only/use_aa=false并移除local_installation/static_edges，不加载地图或安装资料。旧road_rolling_lio.json前端设置不同，不用于当前成对OFF。尚待实际完整运行与共同输入核对。


### 完整L独立输入的FAST-LIO2对照

`20261027T203815Z_656`绑定edf5d73，命令 `data-run -- tools/baselines/run.sh fastlio --source /home/dev/ws_iap/.runtime/0/liuo/20261027T182209Z_414 --scan-model rolling --duration 392.6`。3926云/78521IMU/100381clock，3922输出1.399990234–393.499882812秒，发布墙钟392.601153秒，进程正常结束。原生ring无需恢复，真实time保留。独立shared_input_audit.json通过：冻结原包、原始语义、适配订阅计数及语义一致；暂未传配对LIUO运行，paired_liuo_verified=false。

固定首输出规范位置RMSE2267.705139m（XYZ2170.939970/654.048676/41.552035m），最大4828.193220m，姿态10.448297deg，RPE1s14.597724m/10s146.446609m；沿线/横向/高度2202.677802/537.559787/41.552035m。保留严重失败，不以正常退出宣称算法有效。两场景完整第三方对照均已采集；共同输入与后续完整LIUO运行的配对检查仍待补齐，主方法有效性仍未通过。


S202541候选诊断补充：95.5–100.5秒批次的5个有限候选全部记录31次迭代，达到配置maxIterations=30对应现实现的`iterations()>maxIterations`停止界限。共同参考分离8.621910m是真实记录的数值差异，但日志未保存独立的终止原因/梯度，不能证明多个已收敛物理解或物理不可辨识。候选分析器现保留iterations/iteration_limit_reached及该限制；没有修改估计器、阈值、迭代预算或选支政策。下一步区分数值未收敛与真正镜像，不能仅以有限cost认定物理分支。


### 7站CAL地图的独立L复用提前失败

`20261027T204511Z_773`绑定26df24b，使用独立输入182209、road_anchor_map_reuse.json和201351的7站CAL地图，目标392.6秒。140.5秒实际接入旧map坐标，160.2秒OPTIMIZATION_FAILED：`LOCAL_INSTALLATION_OUTSIDE_OBSERVED_PATCH;linearization=1;along=10.000305`。owner检测子进程退出并终止本run；完整输入审计未完成，不能计作完整复用。未放宽±10m域，失败状态记录optimizer_failure_state.json保留313个变量，无截断，供定位部分重线性化/有限域问题。

1570输出中1374条在接图之前，地图坐标仅196条140.5–160.0秒。继承201351的CAL规范、没有第二次对齐：该19.5秒区间位置RMSE27.514179m（XYZ7.839920/26.333870/1.446702m），最大31.605895m，姿态1.193265deg、速度0.406050m/s；不把短成功输出区间冒充完整结果。160.0秒检查点8站、427个已吸收AA叶、4边，地图位置RMSE82.309055m。叶包含旧先验，本次尚未完成独立精确增量账本审计。最大101组车辆、818因子、RSS388.980MiB、整卡1160.563MiB、优化75.5461ms。

候选终止诊断新增实际LM测试：预算退出不标作收敛、cost目标与自定义停止可并存；加既有启动/镜像/分步优化回归共25项通过，日志`/home/dev/ws_iap/liuo_observation/log/candidate_termination_tests.log`。这是数值诊断检查，不改变分支接纳或物理信息。


### 完整S匹配LIO-only与第三方共同输入

`20261027T205132Z_440`绑定248400e，`data-run -- tools/road_estimator.sh --scene straight --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T191449Z_257 --config packages/liuo_tunnel/config/straight_matched_lio_only.json --seconds 540`。GPU、1×墙钟539.995315秒，5400云/108000IMU/39827UWB/108000clock输入审计通过，5379输出3.1–540.9秒。UWB仅record_only，未加载地图/安装资料，图中无Anchor或UWB因子。FAST202832独立检查重新读取冻结输入，和此OFF的LiDAR/IMU/clock数量及语义hash完全一致，paired_liuo_verified=true；仍不代表尚未完成的S-REUSE已配对验收。

固定首输出规范RMSE1566.505617m，XYZ1566.504489/1.590937/1.000651m，最大2326.386885m，姿态0.452589deg、速度6.261899m/s，RPE1s7.901544m/10s63.001514m。5379输出全部DEGRADED。最大101车辆状态、400因子、RSS294.047MiB、整设备1154.563MiB、队列IMU402/cloud0/UWB0、优化116.034ms；四等时窗RSS中位数282.027/276.863/278.195/281.441MiB。这是完整失败对照，不是主方法有效性证据。


### 新/旧变量混合线性化参考的边界复现

L204511失败前160.1秒已开始移交A9，但尚未写JOINTLY_INITIALIZED，失败状态中含新A9。最小真实ISAM测试以2cm车辆变化和9.99m域内相对点复现新增有限因子时的错误域外线性化：旧版along=10.010000；第二项测试以X/A不同重线性化门限复现既有因子的同类错误。旧失败日志new_finite_chart_before.log、existing_finite_chart_before.log均保留。

修复对新增有限因子及已选中端点的显式有限因子做端点闭包，保持同一当前参考；不扩域、不删因子、不改权重或注入位姿观测。两项新测试加原有流形边缘先验/共同位移回归共4项通过，finite_chart_group_tests.log。日志均在/home/dev/ws_iap/liuo_observation/log；生产L失败区间尚待验证，不能仅凭单测追认原失败已解决。

生产节点构建/安装通过；有限域4项、静态图5项、联合边缘化1项、局部几何14项，共24项回归通过。各suite日志finite_chart_test_*.log和finite_chart_production_build.log、finite_chart_install.log保留。


### 有限域成组重线性化后的L 180秒复用

`20261027T210454Z_857`绑定06fd451，算法836fa17，同一独立输入182209、同一201351地图，180秒GPU1×完成。1800云/36001IMU/10216UWB/46217clock，1779输出3.1–180.9秒，墙钟180.001080秒；未再发生有限域异常。但A9本次没有初始化，因此生产运行没有重现原160.1秒相同插入事件，不能以此代替受控两项回归的因果证据。

140.5秒接图，405地图坐标输出，1374接图前输出单列。原CAL固定规范RMSE32.530552m（XYZ10.104431/30.857183/1.992885m），最大42.188983m，姿态1.277477deg、速度0.472765m/s；RPE1s0.183721m/10s0.761410m。仍不满足完整有效复用。静态精确审计通过：继承240叶，新349实际交付，229融合/120pending、4边；180.0秒检查点保留旧240+新227叶，8站[2,3,5,7,14,15,16,18]，协方差最小特征值7.538702e−6。最终229与检查点227的时刻差按实际图更新顺序核对，不混计。

最大101车辆状态、916因子、RSS407.082MiB、整设备1408MiB、队列421/0/10、优化75.743ms。固定坐标图、账本、输入及资源JSON全部保留；后续继续392.6秒完整验证。


### 成组线性化后的完整独立L复用

`20261027T210852Z_897`绑定ca2625f，实际算法836fa17。同一独立182209输入、201351的7站联合CAL地图，road_anchor_map_reuse.json，完整392.6秒GPU1×结束：3926云/78521IMU/21860UWB/100381clock，3905输出3.1–393.5秒，墙钟392.601034秒。与完整FAST203815的原始LiDAR/IMU/clock逐字段语义hash和数量相同，独立检查paired_liuo_verified=true。

本次81.7秒接图，与180秒运行的140.5秒不同，保留初始化调度/候选不稳定性。786条接图前输出单列，3119条地图坐标输出81.7–393.5秒；继承CAL规范，位置RMSE112.675545m（XYZ22.761699/109.138779/16.322086m），最大218.643447m，姿态2.357599deg、速度0.885878m/s；RPE1s1.400085m/10s6.687042m，沿线/横向/高度15.328452/110.428287/16.322086m。状态INITIALIZING757/LOST1087/DEGRADED2061。全程完成及优于失败对照不能代替有效性验收，仍有明显地图/横向偏差。

精确账本审计通过：旧地图240叶，本轮730实际交付，421融合/309pending，6边；393.0秒后验完整保留旧240+新421叶，10站[2,3,5,7,9,13,14,15,16,18]，最小协方差特征值4.385397e−6。地图离线固定CAL规范位置RMSE77.944479m；完整相关协方差/45对距离诊断保留，不宣称一致性。最大101车辆状态、1117因子、RSS432.297MiB、整卡1194.313MiB、队列421/0/9、优化84.807ms。所有报告和图位于本run export/analysis。


### 成组线性化后的S受影响片段仍无初始化

`20261027T211614Z_498`绑定7fa8c70，算法836fa17，冻结S-CAL190229，straight_local_static.json/straight_local_offsets.json，110秒GPU1×完成：1100云/22000IMU/7080UWB/22000clock，1079输出3.1–110.9秒，墙钟109.995784秒。0A、无地图、350AA全部pending，静态检查INCOMPLETE_NO_EXPORTED_MAP返回非零。固定规范RMSE98.114507m，最大275.852415m，姿态0.366761deg、速度2.935023m/s；不称snapshot定位通过。

候选终止日志9次预算停止、3次绝对cost变化停止；95.6秒批次最后在100.7秒SNAPSHOT_WINDOW_EXPIRED，不能将未完成批次记作已收敛镜像。95.5秒跨窗口相对变化19.889935m，末次完整有限批次未稳定。入口墙面观测9–13秒因LOCAL_WALL_PATCH_NOT_CONTIGUOUS拒绝，14秒才OBSERVED；这是当前完整对称支持要求的诊断事实，不表示已允许入口外推或放宽几何标准。


### L STATIC-PAIR生产比较通过

按预声明150秒执行cache_sync=`20261027T212043Z_828`、continuous_report=`20261027T212350Z_070`，均绑定dca32b7/实际算法836fa17，冻结输入182209与CAL201351地图相同。两组1500云/30001IMU/7979UWB/37980clock的语义hash逐项一致，GPU1×墙钟150.001159/150.000904秒，1479输出。命令采用road_anchor_map_reuse.json、同一--map/--installation、--seconds 150，仅--static-delivery-strategy不同。

独立回调/精确账本均通过，原始257个独立AA叶全部交付；最终新174融合/83pending，150.0秒检查点旧240+新173=413叶完全相同，完整payload和逐边count/weight/weighted_range/weighted_squared_range相等。compare_static_delivery.py返回PASS_SAME_ABSORBED_AA_INFORMATION_DIFFERENT_TRAFFIC。连续组6284记录/3644720字节，缓存257记录/149060字节，序列化载荷减少95.910248%。物理测距计划未改变；不宣称RF/MAC节能。图和JSON在连续组export/analysis/static_pair.png/json。

缓存组140.7秒接图、连续组81.7秒接图，初始化时刻明显不同；协方差和轨迹并不相同，本比较仅证明同一检查点实际AA叶及其标量高斯统计相同，不能证明整个非线性后验等价或定位有效。S STATIC-PAIR和60秒故障/负载矩阵仍未完成。

### S入口原始点夹具

新增extract_snapshot_patch_fixture.py只读取白名单snapshot点云和既有估计姿态/速度，核验冻结原包SHA，按原始索引每4点保留1点并应用已知T_BL；不读真值。local_patch_probe通过生产几何函数诊断，不作为独立验收。S211614的9秒扫描12795点、3199保留点（与生产日志相同），25m内3169点。每侧6个5m区间计数分别[0,0,257,798,148,39]和[0,0,261,801,145,37]；复现LOCAL_WALL_PATCH_NOT_CONTIGUOUS。地面沿向支持约[−1.011,16.580]/[−1.030,16.558]m，未使用Anchor坐标。

夹具与诊断在S211614的export/analysis/snapshot_patch_9p000000.{csv,json}及_probe.json。姿态来自优化后的扫描输出，不是精确记录的在线更新前预测姿态，因此不冒称逐浮点复现。生产支持域尚未修改；后续研究必须限于实际观测覆盖，不能向洞外外推。


### 完整L匹配LIO-only对照

`20261027T212924Z_334`绑定6f75c1f，算法836fa17，冻结独立输入182209、road_matched_lio_only.json，GPU1×完整392.6秒。3926云/78521IMU/21860UWB/100381clock审计通过，3905输出3.1–393.5秒，墙钟392.600834秒。无地图/安装观测，UWB仅record_only、无Anchor/UWB图因子。与完整FAST203815的原始LiDAR/IMU/clock语义及计数独立核对通过；完整REUSE210852也已对同一FAST输入核对，三者共同输入可追溯。

固定首输出规范位置RMSE279.319658m（XYZ246.427437/131.456824/3.477473m），最大674.718286m，姿态5.519472deg、速度2.883587m/s，RPE1s7.261947m/10s29.262993m，沿线/横向/高度245.805030/132.616987/3.477473m；全部3905输出DEGRADED。最大101车辆状态、401因子、RSS400.660MiB、整卡1161.563MiB、队列421/0/0、优化66.981ms。完整对照采集完成，但主方法有效性仍不能仅依靠优于失败OFF判定。

### 入口实测域裁剪的实现与离线验证

按spec预声明替换对称可见性要求：保留零点两侧有支持的连续双墙分量，使用两侧实际沿向min/max交集并裁到原±10m；不跨缺口，不增加地面覆盖或缩小sigma。17项局部几何/候选测试通过，包括新增入口、单侧缺口、中心缺口，旧强曲率拒绝、地面支持及Jacobian继续通过；4项成组线性化/流形边缘先验回归仍通过。日志cropped_local_patch_tests.log、cropped_patch_marginal_tests.log位于/home/dev/ws_iap/liuo_observation/log。

同一S211614的9秒真实点夹具，旧探针REJECTED保留；新探针OBSERVED、墙面域[−1.0435268,10]m、sigma0.127669m，地面区间没有改变。新报告snapshot_patch_9p000000_probe_cropped.json独立保存；它只验证生产几何函数在该夹具的行为，不等价独立几何验收或生产初始化通过。


### 入口裁剪生产结果与地面域一致性修复

S110 `20261027T213914Z_053`绑定2f678ad、算法31e7ef0，使用普通CAL冻结输入190229。完整输入1100云/22000IMU/7080UWB/22000clock，1079输出3.1–110.9秒。入口墙面从9秒可用，但A0仍未初始化；A1在98.2秒由192条TA初始化，350条交付AA全部pending、没有融合。固定首输出规范位置RMSE35.309685m、最大162.758636m、姿态0.304815deg、速度1.315255m/s；仍非有效主线验收。报告保留在该run的export/analysis。

A0正高度候选约−1.042m落在墙面域内却超出同侧地面支持约−1.030m；旧日志CANDIDATE_OUTSIDE_OBSERVED_HEIGHT实际上也涵盖地面沿向越界。最小反例将地面截到±6m而墙面仍支持8m，旧因子在8m错误返回有限代价并允许线性化（floor_domain_before.log两断言失败）。修复将因子的沿向域同时限制在安装侧实际地面覆盖，不扩张支持、不增加高度先验。18项局部测试及4项边缘先验回归通过（floor_domain_after.log、floor_domain_marginal.log）；加入单侧地面裁剪不影响对侧的反例。日志位于/home/dev/ws_iap/liuo_observation/log。正式重放尚待新二进制完成；单元通过不证明A0初始化或整体有效性。


### 地面域一致后的S110生产结果

`20261027T214917Z_388`绑定6c7cb87，同一普通CAL输入190229、straight_local_static.json及straight_local_offsets.json，GPU1×110秒完整结束。1100云/22000IMU/7080UWB/22000clock，1079输出3.1–110.9秒，墙钟109.995637秒。A0于17.9秒、A1于96.6秒各用192条TA联合初始化；350独立交付AA中110实际融合、240pending、1条显式边。110.0秒地图含109叶，最早采集1.012秒，最小协方差特征值1.72365236e−5；独立回调/物理身份/地图账本审计通过。

固定首输出规范RMSE0.326965m（XYZ0.021271/0.166993/0.280297m），最大0.653604m、姿态0.096842deg、速度0.014041m/s；RPE1秒0.020998m、10秒0.037477m。候选日志15次绝对cost变化终止、9次迭代预算终止；仍保留一次相对分支歧义，不将数值停止当可辨识性证明。结果支持继续完整540秒S主路线，不能替代完整S或独立复用验收；受影响rolling检查仍待执行。上述分析在run/export/analysis，未将真值送入估计器。


S离线地图评价现已接通，固定使用CAL坐标规范，无地图拟合。214917的110秒两站地图位置RMSE0.345854m，A0误差0.026538m、A1误差0.488391m；全输入物理/交付各6边，当前地图融合1边。真值来自已匹配冻结scenario布局的既有oracle夹具，并逐条核对物理AA真值距离；只在离线工具读取。3项地图工具回归通过，L旧物理/交付逐边样本数独立核对不变。图与JSON保存在214917/export/analysis/anchor_map_diagnostic.*。

后续完整S运行215143仍在执行，其入口行为已与110秒运行不同：同一算法6c7cb87的候选完成延迟使后续批次从15.9变为16.2秒，跨批相对差异0.609820m超过既有0.5m，A0未初始化。不能将上一短段作为稳定可重复的主路线完成证明；保留完整运行，门限不变。


### 完整S地面域修复运行与数值反例

`20261027T215143Z_916`绑定10f0d2f、算法6c7cb87，完整普通CAL冻结输入190229，GPU1×540秒结束。5400云/108000IMU/39960UWB/108000clock、5379输出3.1–540.9秒、墙钟539.995815秒。A1/A2/A4/A5分别于98.6/190.6/363.6/461.8秒初始化；A0/A3/A6未初始化。1805独立交付AA中624融合、1181pending、2条显式边；540.0秒地图623叶、最早采集1.016秒，最小协方差特征值4.95245783e−6。独立原始交付→回调→地图账本核对通过。

固定首输出规范位置RMSE18.752718m（XYZ14.187035/5.189131/11.111499m），最大98.547116m，姿态0.563777deg、速度0.491200m/s，RPE1秒4.726494m/10秒11.792942m。四站地图RMSE22.197478m；这是非平凡地图与实际AA融合，但尚不能验收有效定位/复用。状态4862 DEGRADED、517 LOST；最大101车辆状态、787因子、RSS357.969MiB、整卡1193.375MiB、队列402/0/4、优化102.188ms。诊断、地图、资源及PNG均在run/export/analysis。入口候选的时序敏感性保留，不将先前S110结果替代本运行。

新增最小数值反例使用实际LocalInstallationFactor和SteppedBootstrapLM：测试专用紧Pose先验、Anchor目标(30,2.8,1)、沿向域±10，初值(9,0,1)。解析受限最优横向为2.8；现有LM终止于0.2588288268，误差2.541171m，1e−3断言失败。日志/home/dev/ws_iap/liuo_observation/log/finite_descent_before.log。它证明无限域外cost可阻断整体步长，使仍有可行横向下降的状态停止；不能把此类cost停止直接当物理分支。反例尚待修复，不放宽域或接受门限，测试先验不进入生产。


有限域求解修复已通过原反例：临时活动集约束步使横向恢复到2.8±1e−3，原观测域不扩张。解析测试覆盖首阻挡面、随后约束释放、平行界、多变量可行性及原图代价不变；局部支持域的X/A Jacobian通过有限差分。无有限域候选仍沿用原LM求解路径。首次接入仅设置params.solver时原失败保持，查明安装gtsam_points通过基类solve虚函数调用，改接该虚函数后反例通过；失败日志constrained_step_tests.log与修复日志constrained_step_dispatch.log均保留。完整初版回归28候选/18局部/4边缘测试通过，生产节点构建与受影响回放仍在推进，不能据此宣布生产初始化稳定。


约束步最终构建/安装完成。新增跨变量解析测试验证约束x≤1时，与x耦合的y由无约束1更新为联合最优1.5；最终29候选/18局部/4边缘测试通过，共51项。日志constrained_step_cross_tests.log、constrained_step_final_local.log、constrained_step_final_marginal.log均位于/home/dev/ws_iap/liuo_observation/log；生产效果待新运行，原有完整S失败保持有效历史记录。


### 线性约束步后的S110生产验证

`20261027T221135Z_418`绑定d117c19，GPU1×同输入190229完整110秒结束，1100云/22000IMU/7080UWB/22000clock，1079输出，墙钟109.995775秒。A0未初始化且18.4秒一次候选快照过期；A1在98.8秒由192条TA初始化。350独立AA全部pending、0融合；地图110.0秒仅A1，最小协方差特征值0.038685738。账本PASS仅覆盖空融合集合及输入身份。

固定首输出位置RMSE38.004384m（XYZ37.992514/0.268968/0.910897m），最大184.961324m、姿态0.295804deg、速度1.361199m/s；RPE1秒19.063868m、10秒42.221843m。数值终止7次预算、13次绝对cost变化；11.3秒候选批仍有1.587978m相对分支差，其中一支相对横向3.799081m、另一支2.800659m。因此解析QP反例通过不证明生产候选已稳定；不能扩大窗口或门限掩盖失败。后续需独立检查线性域内步在联合Pose/Anchor非线性retract后是否越域，并核查其额外计算开销；当前未把该推断当已证实原因。完整报告在run/export/analysis。


### 非线性可行性反例与试探恢复

真实FiniteDomainSolver最小联合旋转/Anchor反例复现：线性预测沿向≤10，实际retract后10.801449257m，LocalInstallationFactor返回Inf；日志nonlinear_domain_before.log两个断言失败。新增试探恢复只处理实际越域Anchor的局部沿向，其余试探状态/横向/高度不变，并重新验证返回delta的实际retract路径。原非线性图决定步接受，不增加观测或prior，不改支持边界/候选门限。

30候选、20局部、4边缘回归全部通过，包含原反例、二阶修正缩放、域内原样、全局变换、重复域明确拒绝。日志nonlinear_domain_candidates.log、nonlinear_domain_local.log、nonlinear_domain_marginal.log位于/home/dev/ws_iap/liuo_observation/log。新增CANDIDATE_DOMAIN_SOLVE_WORK及分析汇总，只覆盖已完成并记录的候选；旧日志标available=false，未完成/过期候选与图线性化等待不冒充已测耗时。生产是否改善仍待新运行。


### 非线性试探恢复后的S110

`20261027T222131Z_969`绑定7b58726，同输入190229及straight_local_static/straight_local_offsets，GPU1×110秒完整结束：1100云/22000IMU/7080UWB/22000clock、1079输出3.1–110.9秒、墙钟109.995665秒。A0于12.4秒由181条TA、A1于97.4秒由192条TA联合初始化；350独立AA中110融合、240pending、1条显式边。110.0秒地图109叶、最早采集1.012秒，最小协方差特征值1.76297830e−5；物理身份/回调/地图账本独立核对通过。

固定首输出规范位置RMSE0.459473m（XYZ0.047726/0.363342/0.277165m），最大0.917151m、姿态0.119647deg、速度0.015629m/s；RPE1秒0.021479m、10秒0.043676m。18个已记录候选共784次成功求解、1866.226ms总求解时间，单候选最大145.543ms；13次绝对cost变化停止、5次预算停止，无分支拒绝或快照过期日志。两批A0正高度候选相对差约7微米、跨批差0.011649m，仍用既有0.5m门限；数值相近不等于统计一致性认证。结果支持扩展完整S540，不能替代全程独立复用或受影响L验收。
