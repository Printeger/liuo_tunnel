# 7 km 曲线公路隧道场景制作结果

日期：2026-10-27（Asia/Shanghai）。**S01–S12 场景与数据验收通过。场景和原始多传感器数据可供后续算法实验使用。**

没有运行 LIUO、FAST-LIO2、LIO-SAM 或其他定位器，没有修改定位算法。
没有执行 APE/RPE、算法排名、初始化、自标定、地图复用或定位去畸变测试。

## 版本与完整数据

启动时为干净 `master/b86933c`，不是需求编写时的 `0abb326`；未 reset、建分支/worktree 或 push。
场景实现提交 `cf36161`，运行期间源码身份保护提交 `0f0b059`。
并行文档工作自行提交为 `6fd5d63`；没有覆盖或代为提交其文件。

**正式数据绑定干净提交 `0f0b059f2b3ca1066d596b65b89d3dec8c642617`**，seed `20261027`。
后续新增独立验收工具和本报告不修改生成器；`metadata/manifests/dependencies.json`
逐文件保存生成源码摘要，生成结束时再次核对。最终文档提交适用性以这些源码摘要为准。

数据根：

```text
/home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968
```

下文 `RUN` 表示该目录。正式 owner 的 `metadata/run_manifest.json` 记录
`formal/protected/completed/pass`、干净源码、`localization=NOT_RUN`。
`metadata/manifests/scene_artifacts.json` 绑定 33 个交付文件的大小和 SHA256；
独立验收逐字重新核验全部 33 个文件，未用退出码替代数据验收。

| 交付 | RUN 下路径 | 大小/摘要 |
|---|---|---|
| 完整原始 MCAP | `export/capture/raw/raw_0.mcap` | 1,888,360,352 bytes；SHA256 `05095c59adad36e908bfb802978e1ecc3ec3b63686755a03c081cb5d40b13005` |
| 录包索引 | `export/capture/raw/metadata.yaml` | ROS 2 MCAP，可独立回放 |
| 200 Hz 真值 | `export/simulation/trajectory.npz` | 21,989,008 bytes；SHA256 `327bdc035ee6fabdb8f8ed59b02fcbdc91180522e01d9e324eff73e9c36d1b6c` |
| 完整碰撞网格 | `export/simulation/scene.ply` | 25,592,122 bytes；SHA256 `5c8888e8b6854ec29aba209f879cf4503345f223f23808b4e69a5d739f4c8f38` |
| 冻结配置 | `metadata/config/scene.json` | SHA256 `832fd527902b8d801ebeaac9dcecb57b4d637da9efe48738797d888c6110ac9f` |
| 安装偏距资料 | `metadata/config/installation_metadata.json` | 仅 ID、侧别、±5.52 m 横向偏距与 synthetic 定义/来源 |
| 独立仿真资料 | `export/simulation/{motion_plan,facilities,anchors}.json` | 运动规划、设施尺寸/位置、站点真值；不进入传感器包 |
| 物理/交付记录 | `export/simulation/{physical_uwb,delivery_uwb,coverage}.csv` | 物理样本、发送源身份/去重、TA 可达集合 |
| 验收证据 | `export/analysis/*.json` | 见下表；附加独立验收不会改写主 manifest |

原始包只含 `/sensors/lidar`、`/sensors/imu`、`/liuo/uwb/observations`、`/clock`。
共 **3926 帧、77,032,491 个有效 LiDAR 返回、78,521 条 IMU、21,860 批 UWB、100,381 条 clock**。
UWB 每批本场景恰好一条观测；没有为了凑射线数输出 40 m 假返回。
保留完整原始包和必要真值作为后续输入，不适用历史 campaign 的 raw release。

## S01–S12 证据

数值容差在生成前固定于 `scene.json` 和[数据合同](spec/road_tunnel_scene_data_contract.md)。
失败后没有放宽阈值。解析积分、有限差分、直段解析射线、网格加密和独立
float64 Möller–Trumbore 线段/三角形测试相互补充，避免只让生成实现自证。

| 条目 | 结果 | 实际证据与边界 |
|---|---|---|
| S01 范围 | PASS | 仅新增模拟器、纯传感器接口/检查器、构建工具和文档；owner/子进程命令在 `metadata/processes/`；无定位器进程或算法输出 |
| S02 全长/断面/轴线/视距 | PASS | 设计轴 7000 m，左右各 150/1200/150 m 回旋/圆/回旋线；独立积分位置误差 <0.1 mm。10.50 m 对称场景限界，5 m 高度检查覆盖 ±5.25 m；拱顶 7.2 m。试验视距模型为反应 2.5 s、制动 3.4 m/s²，约128.18 m沿程采样无遮挡；不是全专业规范认证 |
| S03 设施/无穿墙 | PASS | 路面、检修道、排水沟、电缆槽、14 个灯具支架、7 个凹槽/门面均为碰撞面；设施清单明确实验来源。车辆包络路面不穿透；断面内侧余量充足。接缝、凹槽背板、开口洞口用例通过，1 m→0.5 m 网格对照最大测距差约0.143 mm，无返回集合差异 |
| S04 车辆动力学 | PASS | `geometry_motion_checks.json`、`motion_plan.json`、200 Hz truth。全程前进、右车道、5 s静止首尾；80 km/h，a∈[-1.470589,0.980393] m/s²，max jerk约0.480543 m/s³。p/R/v/a/ω有限差分及连接点连续性通过；合成比力核对 μ=.6 假设 |
| S05 IMU | PASS | `bag_audit.json`：独立速度差分得到的比力最大差 `3.019e-5 m/s²`（门限 .002）；噪声实测约 .0200 m/s²、.0010 rad/s，bias随机游走明示。orientation全零且cov[0]=-1，不提供朝向观测 |
| S06 逐点扫描 | PASS | 每条射线在独立发射时刻求首交；原始点保持瞬时传感器坐标。`rolling_fixture.json`：80 km/h夹具的帧首拼接误差达2.2221 m；测试端真值还原后直段解析平面残差 `2.4e-7 m`。原生ring/time逐点核验；IMU [0,392.6]覆盖全部有效扫描 [0,392.5998877] |
| S07 基站/偏距 | PASS | 18站100+400k、左右交错、普通墙面避开凹槽；墙面±5.6 m、相位中心内移 .08 m。独立积分恢复横向偏距最大误差 `1.74e-6 m`。安装资料未含里程、坐标、真实高度、前后次序；没有加载进算法 |
| S08 三段UWB链路/预算 | PASS | 独立三角形求交核对全部153个AA候选、100个实际物理链路、200个成功/失败报告交付样本；TA/AA/报告者→N0分开。交换间隔≥.004 s，未超250/s。4323个AA物理样本只有730个抵达车辆，未注入全隧道AA图 |
| S09 消息/身份/分包 | PASS | 原共享msg；batch/receive/source分别计时，synthetic共享时钟/raw valid一致。实际软件轮次重发2889条被去重；21860条输出源身份唯一。缺尾分包不阻塞、重复轮次不增样本、同值新帧、NaN序列化夹具通过；稀疏实景每个源帧一条边/一分包 |
| S10 无算法接口/回放 | PASS | 全包Python反序列化和语义hash往返；独立C++ best-effort订阅检查完整收到正式包前2秒20/401/84/485条四类消息。高速短片段另通过20/401/168/569条。native ring和秒time纯字段接口无损，不启动baseline runner |
| S11 真值隔离 | PASS | 全包话题白名单、字段、安装资料键集合核验。真值/网格/站点坐标均在独立文件；纯回放工具只读取原始包。无真值去畸变发布，无新算法先验 |
| S12 全程数据/资源 | PASS | 设计里程0→7000 m，准确运动时长392.576694971 s，完整帧/IMU尾端延至392.6 s。原始包、真值、图、版本/依赖/配置和摘要均实际存在；独立摘要复核通过，资源记录在 `profiling/resources.csv` |

独立附加证据：`export/analysis/independent_acceptance.json`，命令见后文。
它读取保存的 mesh/真值，**不导入生成器的几何或运动实现**。
LiDAR 径向噪声样本44,656个，标准差0.020016 m；TA 21,130个物理样本标准差0.100009 m；
AA 4,323个物理样本标准差0.100418 m，均符合预先规定的统计检查范围。

## 运动、射线与无线模型的具体选择

右车道水平偏置 -1.875 m；车体参考点/三种传感器同址，单位外参。
用实际偏置路径弧长规划，三维车体参考路径长约7000.0875 m，制动起点约s=6685.1891 m。
精确回旋线曲率斜率在连接处跳变；采用固定车体纵坡和0–4.22 mm被动离地高度变化，
使车体p/R/v/a/ω连续。该微小悬架余量是明确的运动学近似，不是标定摆动，未引入完整动力学引擎。

LiDAR合成512×40多线扫描，不冒充Mid-360。完整静态曲线网格常驻，**没有局部缓存裁剪**，
因而不采用world-X块索引，也没有缓存切换假墙；网格离散误差由加密试验给出。
所有原始点未去畸变；仅离线夹具和标明truth的预览使用真值。

AA有效物理边为11条，每边393次（目标1 Hz，含起始样本）；其余候选因LOS/范围不可用。
TA只在可达期间测距，目标20 Hz；全程平均逐边0.9475–4.7784 Hz，不能与可达期间20 Hz混淆。
精确逐边次数及全程平均频率见 `uwb_summary.json`；完整物理事件表可以重算任意时间窗。
全程物理交换25,453次，无预算丢弃；TA覆盖1–4站，零覆盖0 s、可达集合切换32次。

5 Hz软件报告轮次下，730条AA观测成功交付、17,963次报告候选不可达、2889次重复被去重。
三者是报告事件统计，不是独立测距数。源快照最大允许年龄1.2 s，实际接收源年龄最大0.976 s。
固定传输12 ms＋组装1 ms，队列峰8/上限256；本场景每个报告者最多一条定向AA边，
不把不同源帧合并成驱动不可能产生的同一分包。这个模型不是射频/MAC或设备时钟标定。

生成部分墙钟53.45 s，约7.346×仿真速度；包含完整审计、绘图和回放的owner总墙钟235.53 s。
进程峰RSS约1.305 GiB，含双分辨率几何检查；生成阶段资源逐仿真秒记录。
无GPU定位。33项清单总大小约1.942 GB（十进制）；启动和每帧检查8 GiB磁盘预留，未触发限额。

## 图与复现

图在 `RUN/export/analysis/`：

- `route_plan.png`、`route_3d.png`：全路线平面与三维；三维图竖轴缩放明确标记。
- `cross_section.png`：限界、坡面、实际墙面及相位中心。
- `scene_cloud.png`：洞口/中部/终点的真实碰撞几何预览；不是算法输出。
- `motion.png`：里程、速度、加速度、jerk。
- `uwb_coverage.png`：TA覆盖数、逐站可达区间、实际交付AA边。

从仓库目录执行：

```bash
data-run --check
data-run -- timeout 180 tools/build_road_scene.sh
data-run -- timeout 120 tools/road_scene.sh --start 180 --seconds 10 --allow-dirty
# 正式数据命令；启动时工作树必须干净
data-run -- timeout --signal=TERM --kill-after=20 7200 tools/road_scene.sh
# 独立重新验收，使用新报告路径，保留原证据
data-run -- env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/home/dev/ws_iap/road_scene_deps \
  timeout 180 python3 tools/road_scene_acceptance.py \
  /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 \
  --report /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968/export/analysis/recheck.json
```

构建、短片段、完整生成、分析和回放全部经data-run；输出没有写到系统盘。
原始MCAP序列化字节和文件摘要作为本次数据身份固定；重新生成的CDR填充字节/录包元数据
不保证逐字相同，跨重生成应比较所有字段的canonical语义hash及冻结seed/配置。
最后两次高速短片段的四话题语义hash相同。

## 保留的开发失败及局限

开发证据根同正式run根：

- `20261027T133536Z_215`：IMU默认四元数w=1与未知朝向显式清零约定不符；改为全零且cov[0]=-1。
- `20261027T133554Z_570`：CDR填充字节不能作为反序列化再序列化的字段等价判据。
- `20261027T133654Z_922`：UWB float32线格式与序列化前Python float64摘要不同；改为真实线格式语义摘要。
- `20261027T133805Z_131/export/analysis/wire_check.json`：首次DDS发现后过早发布，IMU/UWB/clock初始消息缺失；增加匹配稳定等待后 `wire_check_retry.json`完整通过。
- 最终高速短片段 `20261027T134655Z_583` 完整通过并保留；正式数据另行从干净提交生成，不把开发包充当正式输入。

旧场景、旧算法和旧适配器保持原样。现有LIUO入口及FAST-LIO适配器仍有限制：
rolling支持、初始化、地图/轴线建模等须在后续独立任务处理。本次只交付正确的原始字段和数据，
不宣称这些入口已适配，也不宣称定位通过。

此场景只参考已列主要道路几何条件；设施尺寸/间距、安装高度、车体悬架余量、噪声及无线模型
是公开的实验参数。未做完整结构/消防/通风认证、真实Mid-360仿真、RF/MAC仿真或实车轮胎动力学。

重复开发中间物已按用户授权退休：`133215Z_610`、`134201Z_141`、`134415Z_381`、
`134922Z_454`（均带日期前缀20261027T）的重复MCAP/网格，共203,026,424 bytes。
每个目录保留逐文件SHA256/大小/退休状态清单；失败证据、报告、图和真值未删除。
正式完整包和最终高速短片段原始包均保留。汇总在正式run的
`export/analysis/retired_development_intermediates.json`；它不改变历史主manifest或原摘要。
