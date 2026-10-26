# UWB 观测接口统一：持续开发 Goal 启动 prompt

将下面整段发送到新的开发会话。本文只是启动文本；生成或阅读本文不等于已经启动 Goal。

```text
请创建并启动一个持续开发 Goal：将 src/liuo_tunnel 的仿真 UWB 发布、估计器输入 topic 与生产回调统一为当前 liuo_uwb_msgs/msg/UwbObservationBatch，默认 /liuo/uwb/observations；同步迁移构建、录包、回放、审计与相关工具，完成针对性测试及代表性仿真回归，并交付可复现证据。

工作区：/home/dev/code/ws_tunnel。所有项目修改限于 src/liuo_tunnel；src/nlink_parser_uwb 及 IAP/demo 只读。依赖构建产物允许写数据盘。不要新建分支/worktree，不 push，不覆盖用户改动，不回退新增工作。本 Goal 未指定 token budget。

先读取：
- 适用的父目录规则和 src/liuo_tunnel/AGENTS.md。
- src/liuo_tunnel/docs/uwb_observation_unification_requirements.md（本次主需求与完成条件）。
- src/nlink_parser_uwb/docs/liuo_integration_contract.md。
- src/nlink_parser_uwb/packages/liuo_uwb_msgs/README.md 及 msg/UwbObservation.msg、msg/UwbObservationBatch.msg。
- src/nlink_parser_uwb/docs/uwb_driver_design_notebook.md 第 10–14 节，ros2_migration.md、software_acceptance.md 的相关边界。
- src/liuo_tunnel/docs/spec/deployment_initialization_contract.md、run_artifact_contract.md。
- 为选择最小回归，读取阶段三代表性结果和当前相关配置；不先重跑全部历史实验。

核查实际 Git HEAD、branch、worktree、用户修改及现有运行。编写需求时 LIUO 为 master/8d3ed64，驱动为 a13dc6dc26ff7fed11272c2ec9a9f29020805b9a；这些是核查参考，不是 reset 目标。当前阶段二/三结论与历史失败原样保留，本次仅验收接口统一。

先给简短计划，然后立即实施，持续到全部必需工作完成，不停在设计或承诺上。

沿一条主线推进：
1. 固定接口来源、逻辑 ID 映射、时间接受策略与噪声/相关性模型；打通数据盘构建、消息 underlay 和运行路径。
2. 实现实际 LIUO batch 回调：逐条校验、适配、保留原始元数据、身份去重和入队；图变更只在估计器更新中进行。
3. 修改真实 simulator 发布与 capture，让同一回调接收；迁移 observer、replay、hash、delivery fault、visualization、evaluation、启动 readiness 与 manifest。
4. 完成单元边界、公开 ROS 输入集成、录包/回放、60 s 软件负载和代表性 GPU 仿真/地图复用回归，提交实现与结果。

必须落实的合同：
- 只依赖现有 liuo_uwb_msgs，不复制消息、不改共享 .msg、不用旧 UwbRange topic 中转冒充完成。新 topic 可配置/remap，默认 best_effort/volatile/keep_last100；配置变化写入元数据。
- Batch Header 是发布时刻，不是测量时刻。逐条使用 source_frame_stamp/time_basis；received_stamp 是驱动接收时间，LIUO callback 时间另记。不用 Header、received 或 now 填未知测量时间。
- TA time_basis=unknown/receive-only 默认只记录、不融合。mapped 也须有合法时间和可追溯同步/时间误差依据；NaN uncertainty 不能当 0。实机缺依据不偷用仿真配置。
- 本轮 AA 时间未知也默认记录并拒绝，无需新建无时间静态融合框架。已接受 AA 仅约束固定基站；不是当前车辆姿态因子。
- 方差从 LIUO 显式模型获得，新消息无 variance_m2。合成基线 σ=0.10 m / variance=0.01 m² 标为仿真假设，不能宣称实测噪声。RSSI 未知保持 NaN/false，不等同 LOS/NLOS。
- freshness=false 不证明独立测距；缺独立性依据时默认不添加独立因子。若支持相关报告，必须实现有界保守模型并验证。新源帧、发送轮次和新 batch 均不自动等于新 TWR。
- 双向 AA 可能同源；审计现有 summary 累加精度，避免副本加倍和不受控信息增长。保留报告方向与来源，不能强制仅小 ID 报告者而丢弃单端覆盖边。
- 原仿真 tag 0 与 anchor 0…6 有 ID 冲突。采用显式表 N0→tag0，N1…N7→内部 anchor0…6；图/地图/安装文件保留内部 ID。生产代码不能由逻辑 ID 算坐标、顺序或间距。网络名来自配置，不存在于 msg 中；默认一个入口一个网络。
- 观测身份包含 network、driver_session、reporter、source_session、有效 source_frame_seq 和边端点/kind。同帧多个 peer 都要接受；换 tx_cycle 重发同帧同 peer 不新增信息。batch_seq 不替代测量身份。
- 覆盖 32 位源序号回绕、乱序、driver/source 重启、旧会话迟到、缓存退休和 ROS 时钟回退。所有缓存有界，清理后旧包不能复活。不要截断 64 位 session 或用到达计数制造新测量。
- 本地新帧/远端已到 shard 立即处理，不等齐分包。正常本地 batch 不套用无线每包最多 5 边限制；批级非法与条目非法分别处理。

仿真边界：
保持当前七站/400 m、800 m 三维量程、LOS、σ=0.10 m 和独立噪声流；真值只用于生成/评估。保留原 TA20Hz/AA1Hz 目标及250交换/s的物理调度用于等价回归，不能暗中改成5Hz新独立AA测量。
可增加明确的软件报告场景验证50Hz本地帧及5Hz远端完整轮次，但与物理基线分开。重发旧快照保留源帧身份，不能复制旧距离补“新测量”。实际生成独立样本时仿真可有可证明的精确时间/freshness=true，另测驱动真实语义下的 NaN/false/unknown。
一帧多边只能共享其真实声明的源帧时间，不将不同时刻的距离偷偷同步。核对 distance_m 的 float32 误差；毫米量化若引入需单独声明。

运行链要求：
- 新包直接记录/回放统一 topic，不与 bridge 再生成或 live 发布双发。batch数、观测数、唯一报告数和融合数分别统计。
- hash 包含全部字段与稳定 NaN 表示；新旧格式不同不要求字节 hash 相等，物理等价对比需显式端点/身份映射。
- 延迟/重复保留源身份及时间；单 anchor fault 仅过滤相关条目，其他同 batch 条目继续。NLOS 测试偏置与标签留在测试日志，不向不存在的 variance/status 字段赋值。
- 历史旧 bag 不覆盖。最小范围可明确拒绝并说明使用历史版本；如实现转换，要保存来源、假设及新旧映射，不伪造质量/同步证据。
- use_aa=false 和 UWB OFF 继续可用；校准地图不重复吸收旧观测/安装信息，现有图和坐标规范保持。

构建与存储优先处理：
先 data-run --check。所有生成构建/测试/回放/分析产物的命令经 data-run -- ...，输出在 /home/dev/ws_iap。当前脚本有项目内 build/install/log 及二进制路径硬编码，必须把受影响路径贯穿修好；不能绕过 launcher 或回退系统盘。
解决 env -i 清理 underlay 的问题，显式 source 唯一消息依赖和目标 install。更新源码/接口/二进制/配置身份记录；一个 LIUO_RUN_DIR，一个 owner 终结 manifest。使用已有超时、无进度及磁盘保护，仅管理本任务进程。保留失败和被引用的报告/图，不删除当前输入；过期中间数据按现有保留规则处理。

最小验收按需求书 T01–T14 执行：
1. 干净环境构建和类型/QoS；多 peer、多 shard、缺片即时处理，ID 冲突与非法条目测试。
2. 时间未知/NaN/错误、噪声缺失、freshness/双向AA、重发/回绕/重启/乱序/退休等行为测试，包含真实回调和图侧计数断言。
3. simulator→统一 topic→生产 LIUO 实际融合；driver-like 合成输入正确记录/拒绝。独立 demo subscriber 不能代替这项。
4. 新包记录与原样回放身份/语义一致；故障注入、时钟重置、开关、工具可用；60秒负载报告逐边率、10秒窗口及资源边界。
5. 使用已有代表性配置完成足以初始化的仿真，并验证自标定→导出→独立复用受影响路径，可合并运行证据。短段不足以建图则使用现有代表性路线所需时长。比较迁移前后物理报告、关联与因子、输出状态；不以同seed代替同输入。
GPU先做设备/驱动预检。CPU证据仅为诊断，不能冒充GPU通过。不重做十种子/全故障矩阵，不新增精度门槛；现有弱高度、初始化失败与过度自信如实披露。只运行改动所需检查，发现新问题才扩大。

持续自主执行：
- 本范围内接口/配置/数据结构/可逆修复和必要测试已授权，自主决策，不逐项索要确认。
- 实验失败就定位第一个违反的合同，修复并继续；不隐藏失败、不降低测试要求、不通过给未知字段填高置信值制造通过。
- 实机缺标定或设备不是完成独立软件工作的阻碍；单列未验证边界，不把软件未完成冒充“仅待硬件”。
- 不扩展无线固件、全新估计器、NLOS分类器、全面精度优化或跨仓库修改；只有真正需要新增现场承诺或扩大范围时才问。
- 定期简短更新已实现内容、证据和下一步；不在每次实验后最终收尾。
- 本地按检查过的逻辑变更提交，正式live证据绑定干净已提交版本；不push。
- 未完成实现、运行链和必需测试不得complete；总体无法推进时按Goal工具的阻塞规则处理，不把单次失败当整体阻塞。

交付代码、配置与测试，更新README可执行命令，并新增：
- docs/spec/uwb_observation_input_contract.md
- docs/uwb_observation_unification_results.md
结果列明提交、接口版本、命令、数据盘证据、每项通过/失败/未执行及限制。最终结论仅限已验证的软件接口统一与代表性仿真，不宣称真实无线、硬件时钟或全场景部署认证。

现在创建Goal，给出简短实施计划并立即开始。
```
