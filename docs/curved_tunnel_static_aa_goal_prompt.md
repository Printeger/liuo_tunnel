# 曲线隧道与静态 AA 新版 Goal 启动 prompt

将下方整段复制到开发会话。本文只是启动文本，生成本文不代表 Goal 已启动。

```text
请创建并启动一个持续开发 Goal：依据 curved_tunnel_static_aa_design.md 和 curved_tunnel_static_aa_requirements.md，开发 LIUO 的曲线隧道/rolling/静态 AA 地图新版，并在现有 2.4 km 短直隧道和 7 km 长曲线隧道完成代表性算法验证、自标定→相关地图导出→独立复用，以及 FAST-LIO2 第三方对照。

本次明确授权算法开发和运行验证。此前“只生成场景、不运行算法”的约束属于已完成的场景制作任务，不限制本 Goal；但不得覆盖旧场景、冻结输入和历史结果。本 Goal 不设置 token budget。

工作区 /home/dev/code/ws_tunnel；项目修改仅限 src/liuo_tunnel。IAP/demo、src/nlink_parser_uwb 和原 liuo_uwb_msgs 只读。先检查适用父目录规则、实际 HEAD、工作树和进程。需求编写基线 master/dc56f57 仅作参考，不 reset、不覆盖用户修改、不建新分支/worktree、不 push。

先读：
- /home/dev/AGENTS.md 与 src/liuo_tunnel/AGENTS.md。
- docs/curved_tunnel_static_aa_requirements.md：本 Goal 主范围、C01–C16 和完成条件。
- docs/curved_tunnel_static_aa_design.md：几何、物理身份、唯一计权与最小设计。
- docs/spec/deployment_initialization_contract.md、docs/spec/run_artifact_contract.md。
- docs/road_tunnel_7km_scene_results.md、docs/spec/road_tunnel_scene_data_contract.md。
- docs/uwb_observation_unification_results.md、docs/phase3_representative_results.md 的当前结论；需要追查具体失败时再读历史细节。
- 只读的 nlink_parser_uwb 新 msg/接入合同，以及当前生产节点、AA摘要、初始化/地图、模拟器、run/replay和FAST-LIO2接入代码。

给简短实施计划后立即开始，实现生产流程和针对性验证，不停在规划。先走一个小型未知 Anchor＋静态边＋地图往返闭环，再扩展到两场景完整验证。

冻结输入事实：
1. S 是当前空矩形直隧道，世界长2400m、约6×5m、7站、snapshot 10Hz/IMU 200Hz。普通outbound约5m/s，x=0→2350m约530s，加尾段使用约540s。360s calibration-route只到x=1000且含三维激励，只能作旧流程回归，不能冒充完整S或汽车运动。
2. L 是已制作的7km双车道左右缓弯道路隧道，静止启动、正常变速最高80km/h，rolling 10Hz/IMU 200Hz，数据到392.6s；18站、已测物理AA只有11条。合法横向安装偏距±5.52m，不是建筑限界半宽；高度、里程、道路轴真值不进入估计器。
3. L冻结run为 /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968；原始包 export/capture/raw，生成提交0f0b059f2b3ca1066d596b65b89d3dec8c642617，seed20261027。先核对manifest/文件摘要，不重生成来掩盖问题。
4. 旧S的新msg首轮/复用包为 /home/dev/ws_iap/.runtime/0/liuo/20261027T031155Z_389 和 20261027T031819Z_304，可作回归。完整普通S不足时用现有场景另录，标明新输入。
5. S/L LiDAR topic、外参和scan model不同。L用/sensors/lidar、原生ring和相对秒time、单位外参；旧S使用/sensors/lidar/points及非单位外参。配置、observer、回放、评估和FAST-LIO必须统一接通，不能只改节点订阅。

算法原则：
- 保留原始GPU VGICP、IMU、TA/AA联合图和X/V/B/A。不得把LIO输出/IMU预测位姿当独立观测再次计权。
- 核心距离因子无需改成“曲线距离”；必须解除未知Anchor启动、候选、安装/墙面与地图格式的全局直线依赖。不能仅关continuous_installation_walls就宣布支持。
- 首轮无Anchor坐标，无已知高度/顺序/里程/轴线半径，无车辆真值起点/yaw/离地高度；ID不蕴含位置。已知外参、静止启动、有来源的横向偏距和实际局部几何可用。
- 允许有限候选、延迟初始化、镜像/弱高度与诚实状态。车辆估计可以继续，但旧TA参与跨窗口初始化时必须保留车辆不确定度/相关性，不能将缓存位姿当精确已知量，也不能无限保留启动历史。
- 先不增加7km全局样条、每站自由轴或分布式地图。若实际失败证明需要局部墙面/断面，定义观测来源、关联、有效跨度、曲率误差与相关性后实施；不把当前局部切线套到远端Anchor。
- rolling必须使用真实点时间和IMU/估计状态进行运动补偿，定义状态参考时刻、坐标及外参。禁止清零time、真值去畸变、用snapshot冒充rolling。审查高速TA插值；保留snapshot回归。

静态AA主线：
1. 原始batch继续使用原msg。TA走车辆时间队列；AA按物理部署/来源有效性进入StaticAnchorEdges等小型Module，不绑定车辆lag。回调只校验/入队，图仍由估计器更新。
2. 定义network、deployment_epoch、无向端点、稳定measurement/block ID、revision、精确covered_blocks、原始来源/误差模型。物理身份必须跨重传/驱动重启稳定；交付session/frame不是物理测量身份。
3. 如原msg不能表达缓存摘要，新增项目内最小独立静态记录合同和生产接入，不改共享msg，不把均值伪装原始测距，不改写采集时间，不借新session复活旧测量。
4. 优先复用现有count/weight/mean/scatter与AA残差。一条边保留一个尚未被prior吸收的显式累计因子；重复/双向/转发不加权；独立增量可合并；累计版本含旧块时验证覆盖后替换。样本集合不同不自动证明误差独立。
5. 最多保留S=7/L=18个已经合法初始化的持久A；旧车辆状态边缘化时纯AA不淘汰，不逐帧重加。端点未知则有界缓存，不凭空初始化。
6. 保存联合地图及精确已吸收清单为一致检查点。加载后同块只去重，新独立增量才入图。不能删显式factor就声称从稠密prior撤销旧信息。旧v1地图无精确账本时保守拒绝无法证明不重叠的AA，不捏造迁移身份。
7. 时钟重置与Anchor移动分开；同部署合法静态记录可保留，驱动新dataset不能增加旧块权重。Anchor移动/更换/ID重用需新有效时期与明确重建，不能只改一条边继续信任旧地图。
8. 生成“冻结有限物理块＋发现/缺失/版本变化同步”的可选策略。初访仍能获取记录，遵守AA测距与报告到车的实际LOS；不能把全局physical_uwb日志绕过交付直接给机器人。旧长包1.2s快照合同和原包不改写；旧缓存交付另建衍生数据和身份。
9. 对同一物理块集合比较连续重发与缓存同步的权重/信息及交付负载；减少物理采样的策略另作不同信息实验。报告延迟/次序改变可影响在线估计，不强求所有轨迹逐帧相同。

验证顺序和最小矩阵以需求书为准：
- C01–C12做必要单元/集成和真实生产回调/图侧验证。重复1000次不增信息、显式高斯图与摘要cost/梯度/Hessian等价、旧AA超lag真入图、未知端点后续入图、替换/已吸收前缀、图更新失败/检查点恢复、时钟与部署重置均覆盖。
- 先S旧包短回归和小闭环，再L高速/弯道片段，最后冻结版本运行完整S和L；短片段不能替代完整路线。
- 每场景首轮未知Anchor完整主路线→真实地图导出→独立噪声/新进程/重新车辆初始化的完整复用。S至少一例不同起点或朝向；L可保持正常汽车路线并生成新噪声。不得只重放同包换run ID冒充独立第二轮。
- 对各自REUSE输入运行LIUO LIO-only和FAST-LIO2，严格核对共同LiDAR/IMU。对CAL输入各做一例有运动及实际AA入图的TA-only有限消融。不要全组合、多种子或扩展LIO-SAM。
- 两场景各一段STATIC-PAIR检验同物理集合的交付方式；一套至少60秒生产负载/慢订阅/故障测试检查资源、逐边统计、去重和恢复；两种scan model补受影响冒烟。
- FAST-LIO2旧runner拒绝rolling、限制360s、写旧路径、固定旧topic/外参，必须真正适配。核查原生点时间单位和snapshot配置，不仅删拒绝条件；不使用UWB/真值/LIUO地图。第三方算法真实失败如实保留，接入错误不能算有效对照。
- 保持实际GPU、1×原时间回放和有界资源；CPU/减速仅诊断，不能代替正式证据。不要求AA独立图刚性或全部Anchor初始化，但S/L都要有实际未知Anchor估计和非平凡地图复用，L需包含曲线运动段。全程pending、纯LIO或只读写地图JSON不算完成。

评估与证据：
正式运行前冻结数值容差、输入、配置、规范和资源预算。主评价固定坐标规范，复用不按第二轮真值起点或整条轨迹重新对齐；补充对齐指标必须单列。报告输出覆盖、分轴/沿线误差、APE/RPE、姿态/速度、初始化/状态、地图/间距/弱方向、物理/交付/入图AA集合、延迟/资源/通信量。
已有精度参考不是硬门槛，不新增宽松阈值伪造通过；不因OFF灾难性失败就称主方法有效。未知高度和镜像诚实报告，不隐藏未输出或失败区间。单run PASS不等于阶段完成。

执行与存储：
先data-run --check。所有构建、测试、生成、回放、分析通过data-run -- ...，产物只在/home/dev/ws_iap；修正硬编码/tmp/源码树路径，不绕过隔离。每run一个owner和LIUO_RUN_DIR、原子manifest、独立ROS domain和唯一clock。保留总超时、无进度、队列/候选/缓存和磁盘保护，只清理本任务子进程。
原包与当前任务输入不删不覆盖；收尾按主机规则退休无需再用的重复中间物，保留报告/图/摘要、可复用地图和必要复现输入，登记实际可用性。
按检查过的逻辑变更本地提交，正式实验绑定干净提交及实际二进制，不把旧运行冒称新版本。只重跑受改动影响的检查。

持续自主推进：必要实现、可逆修复和实验已经授权，不逐项索要确认。单次失败继续诊断；同一假设连续两次无支持时换有证据的路径。若确实需要新增物理信息/改变场景，明确说明依赖而非偷加先验；参考精度没达到或一条路线失败不等于整体阻塞。遵守Goal工具的阻塞规则，不以预算/疲劳或部分完成标complete。

交付：代码/必要测试、S/L配置和完整可执行命令、FAST-LIO2接入、地图与静态边检查点、docs/spec/static_anchor_edges_contract.md、docs/spec/curved_tunnel_estimator_contract.md、docs/curved_tunnel_static_aa_results.md及docs/curved_tunnel_static_aa_evidence.json、数据盘审计/对照图、README当前使用说明。按C01–C16逐项给证据和局限。

只有生产能力、双场景主路线与有效地图复用、第三方对照及信息/资源验收全部达到需求，才能complete。最终结论限定代表性仿真，不声称任意启动、精确高度或现场部署认证。

现在创建Goal，给简短计划并立即开始开发。
```
