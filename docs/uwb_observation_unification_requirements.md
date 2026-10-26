# UWB 观测接口统一任务需求书

日期：2026-10-26（Asia/Shanghai）。状态：待实现；本文不代表迁移或测试已完成，也不启动开发 Goal。

启动文本：[持续开发 Goal prompt](uwb_observation_unification_goal_prompt.md)。

## 1. 目标与范围

将 LIUO 的仿真 UWB 发布、估计器输入 topic 与回调统一到已经实现的 `liuo_uwb_msgs/msg/UwbObservationBatch`，默认话题 `/liuo/uwb/observations`。仿真、真实驱动输出和新格式录包通过同一个生产回调完成解析、校验、身份管理及入队；估计器更新线程继续负责图修改。完成构建、接口/异常测试、真实 ROS 回调集成测试和代表性仿真回归。

“统一”既包含消息类型，也包含时间、逻辑身份、原始距离、质量有效性及逐条处理语义。不得把新消息转换成旧 topic 后让生产估计器继续只订阅 `UwbRange`，也不得为仿真另设绕过生产校验的入口。内部可保留或扩展 `RangeObservation`，不要求优化器直接依赖所有 ROS 字段。

本次为接口迁移，不是重新开发 LIUO 算法或重新验收全部历史阶段。保留原始 GPU VGICP、IMU、UWB 的联合图、自标定、地图相关信息、地图复用及 `use_aa` 开关；不改变当前默认 AA 关闭的选择。数据不具备融合条件时可记录并明确拒绝融合，但不能用“全部拒绝”代替合格仿真数据的端到端融合验收。

## 2. 权威来源与已核查基线

优先级：实际 ROS2 `.msg` 定义 → 当前消息说明及接入合同 → 当前 ROS2 实现/验收 → 原始设计 notebook。Notebook 中标为“拟新增，ROS1”的内容是设计来源，不覆盖现行 ROS2 字段。

- [UWB 接入合同](../../nlink_parser_uwb/docs/liuo_integration_contract.md)。
- [消息说明与字段语义](../../nlink_parser_uwb/packages/liuo_uwb_msgs/README.md)、[单条定义](../../nlink_parser_uwb/packages/liuo_uwb_msgs/msg/UwbObservation.msg)、[batch 定义](../../nlink_parser_uwb/packages/liuo_uwb_msgs/msg/UwbObservationBatch.msg)。
- [驱动设计 notebook](../../nlink_parser_uwb/docs/uwb_driver_design_notebook.md) 第 10–14 节、[ROS2 需求](../../nlink_parser_uwb/docs/uwb_ros2_development_requirements.md)、[软件验收](../../nlink_parser_uwb/docs/software_acceptance.md)、[迁移说明](../../nlink_parser_uwb/docs/ros2_migration.md)。
- 本项目 [AGENTS.md](../AGENTS.md)、[部署初始化合同](spec/deployment_initialization_contract.md)、[运行产物合同](spec/run_artifact_contract.md)。

编写时 LIUO 为 `master`，HEAD `8d3ed64`，工作树干净；驱动 HEAD 为 `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`。后续实现先记录实际 HEAD 和接口文件摘要，不回退期间新增工作。历史阶段的完成范围和失败继续保留；驱动软件通过不等于 LIUO 已接入或无线硬件通过。

| 当前实现 | 迁移要求 |
|---|---|
| `liuo_tunnel_msgs/msg/UwbRange`，`/sensors/uwb/range`，每条边一消息 | 新类型与默认 topic 成为唯一默认生产 UWB 入口；按 batch 中各条观测处理 |
| `header.stamp` 是有效测量时间 | 使用每条观测的时间依据；batch Header 只表示组装/发布 |
| 消息自带 `variance_m2=0.01` | 方差由 LIUO 显式噪声模型提供，记录来源；新消息没有此字段 |
| `producer/session/sequence` 标识单条边 | 保留网络、数据集、源会话、reporter、源帧及端点复合身份 |
| tag ID=0，anchor ID=0…6，以类型区分 | 新逻辑 N0 为机器人；显式映射 N1…N7 到既有 anchor ID，不直接复用 0…6 |
| 原模拟器 TA 目标 20 Hz、AA 目标 1 Hz、总交换预算 250/s | 接口等价回归保留物理调度；5 Hz 远端报告另作明确的协议负载场景 |
| 录包、observer、可视化、hash、回放和故障注入写死旧字段 | 同步迁移整条运行链，分别统计 batch、观测和入图数量 |
| 构建/启动脚本清理继承环境，只加载本项目安装目录 | 显式装载唯一 `liuo_uwb_msgs` 依赖；调整数据盘 build/install/log 路径及二进制定位 |

当前物理世界实际为七站/400 m、800 m 三维量程、LOS、σ=0.10 m；README 部分历史段落仍描述旧布局，不能据此把本任务改为 49 站。编号不是坐标信息。

## 3. 外部消息合同

### 3.1 Batch

类型为 `liuo_uwb_msgs/msg/UwbObservationBatch`，字段完整保留：`header`、`driver_session_id`、`batch_seq`、`receiver_id`、`trigger_type`、`observations[]`。

- `LOCAL_RANGE=0`：本地测距事件；`REMOTE_EDGE_REPORT=1`：远端分包事件。默认接收者为逻辑 N0。
- 默认使用 best effort、volatile、keep last 100，与驱动兼容；深度可以显式配置并记录，不能靠不匹配的 reliable 订阅导致“没有数据”。topic 支持统一参数或 ROS remap，发布、订阅、录包和工具必须一致。
- 本地新源帧、远端已到的有效分包立即处理，不等其他来源或剩余分包。不得用统一 5 Hz 定时器拖慢 TA。
- Batch 不是同步快照。单个本地源帧可以含多个 peer；远端同源帧可跨多个 shard。无线 envelope 每包最多 5 边不是所有 ROS batch 的最大长度。
- 空 batch 不造成异常或图更新。批级身份/枚举非法时可整批拒绝；单条非法时应记录原因并继续处理其他合法条目。显式限制可接受批长度及缓存容量，超限行为有计数。
- `header.frame_id` 依当前合同为空；距离不提供坐标变换，不能从 Header 推断地图系。

### 3.2 单条字段与旧表示的关系

| 字段组 | 必须遵守的含义 |
|---|---|
| `kind/reporter_id/node_a/node_b` | TA 为 `ROBOT_ANCHOR=0`，AA 为 `ANCHOR_ANCHOR=1`；按逻辑注册关系验证，远端 node_a 应与 reporter 一致 |
| `source_session_id/source_frame_seq/source_frame_seq_valid` | 源会话属于 driver 数据集；源帧序号不是逐对 TWR transaction ID |
| `tx_cycle_seq/shard_index/shard_count` | 传输元数据，不能作为唯一测量身份；不等待分包齐全 |
| `source_local_ms/source_system_ms` 及两个 valid | 原始设备毫秒，可能回绕；不是 Unix 时间，不在 LIUO 中再次猜测映射 |
| `time_basis/source_frame_stamp` | 源帧时间及其有效映射状态，不是无条件精确的逐边 TWR 时间 |
| `received_stamp` | 驱动完成输入 read 的 ROS 时间；保留为驱动接收时间 |
| `source_hold_age_ms/source_hold_age_valid` | MCU 持有年龄，仅覆盖链路的一部分；无效值不能拿来算端到端年龄 |
| `time_uncertainty_s` | NaN 表示未知，不等于零 |
| `distance_m` | float32，米；转内部 double 不恢复 float32 已损失的精度 |
| `fp_rssi/rx_rssi/rssi_valid` | 未知为 NaN/false，不伪造高质量；不能直接等同 LOS/NLOS |
| `pair_measurement_freshness_verified` | 驱动目前为 false；新帧或新 batch 不证明独立测距 |

旧 `status`、`propagation_status`、`variance_m2` 在新消息中没有直接对应字段。无数据、超时、失联通过缺流/诊断表达，不发布零距离；NLOS 注入标签保留在测试元数据，不能偷偷扩展共享消息定义。

## 4. LIUO 适配策略

以下是本次实施的默认边界；有代码或测试依据时可细化实现方式，在结果报告记录，不以修改共享协议来规避。

### 4.1 ID 与依赖

1. 直接依赖相邻仓库提供的 `liuo_uwb_msgs`；在 CMake、package.xml、Python 环境中声明并加载。不复制 `.msg`，不在 LIUO 中再定义同名包。
2. 所有项目修改限于 `src/liuo_tunnel`；驱动及消息包作为只读依赖。可从其指定包路径构建消息 underlay，构建产物写数据盘，无需改动驱动。
3. 配置中明确 `network_instance` 与逻辑节点到内部 typed ID 的映射；同一入口绑定一个网络，不能猜测消息中不存在的网络字段。多网络混合输入必须可区分，否则拒绝而非混同。
4. 代表性仿真采用显式表：N0→tag 0，N1→anchor 0，…，N7→anchor 6。生产代码读取表，不通过 `logical_id-1` 或设备原始 ID 推算坐标。注册表版本/摘要进入 run 元数据。
5. 地图、安装输入和图键继续用内部 ID，保持历史地图兼容。未知、重复映射、非固定 AA 端点、自环和不一致 reporter 应拒绝；不得通过“ID 看起来像数组索引”接受。
6. 驱动当前 raw ID 19 映射为逻辑 N0，说明 raw ID 和逻辑 ID 不相等；LIUO 不重新解析或截断 raw ID。

### 4.2 时间与有效性

| 情况 | 默认行为 |
|---|---|
| TA，local/system mapped，源时间合法，配置有可追溯同步/帧时间误差依据 | 按源时间进行现有固定滞后关联；记录帧级近似与时间误差处理 |
| TA，`TIME_UNKNOWN=0` 或 `TIME_RECEIVE_ONLY=3` | 记录但不进入动态因子，不用 received/Header/now 补测量时间 |
| mapped，但时间不确定度为 NaN 且无外部有效误差界/模型 | 记录并拒绝动态融合；“映射成功”本身不足以证明可融合 |
| AA，时间合法、噪声及独立性策略满足条件 | 进入固定 A–A 处理；不生成当前机器人位姿的测距约束 |
| AA，时间未知 | 本轮默认记录并拒绝入图；无需为了接入而新增无时间静态融合系统 |
| 无有效源序号 | 可记录，默认不融合；不生成新的 arrival sequence 伪装新观测 |

`TIME_FRAME_LOCAL_MAPPED=1`、`TIME_FRAME_SYSTEM_MAPPED=2` 是必要信息，不是自动放行标签。接受配置需说明适用数据来源、模式、时钟域、时间误差上界或模型以及误差过大时的拒绝条件。允许已标注的仿真精确时间配置；不能把该配置默认用于实机。

若后续确有需要实现未知时间 AA 静态路径，必须独立保留“测量时间未知”、地图/安装版本和接收调度时刻，定义过期/重放/设备替换行为并测试；不可向通用队列填入伪造测量时间。该扩展不是本轮完成的前置条件。

保留 `received_stamp`，另外记录 LIUO 回调收到消息的时间，分别统计驱动接收→回调延迟和已知条件下的源帧年龄。回放只复用原消息时间；ROS 时间回退、源重启和新 driver 数据集需明确清理/退休，不让旧数据挂到新车辆轨迹。未来数据有限等待，过期数据明确拒绝。

### 4.3 方差、质量与 AA 信息

- 噪声由 LIUO 消费配置提供，至少包含模型标识、适用来源、TA/AA 标准差、单位和依据。非法/零/负/非有限参数拒绝启动。
- 仿真模型使用现有 σ=0.10 m，即方差 0.01 m²；在配置和结果中注明“模拟器噪声假设”。它不是驱动给出的方差，也不是实测标定结果。
- 实机缺噪声/时间/独立性依据时保持记录及明确拒绝，不能继承仿真默认权重。时间误差带来的关联误差应有明确接受上界或建模，不能把未知误差当零。
- freshness=false 不等于坏距离，但不允许仅凭持续新源帧按独立高斯观测无限累计信息。默认缺乏独立性依据时记录并拒绝独立因子；若配置允许相关原始报告，则必须给出有界保守模型及测试。
- AA 以无向几何边管理信息，同时保留有向报告来源。A→B、B→A 不默认视为独立两次测量；同一测量双向转发不使信息翻倍。既有 AA summary 中逐次加精度的逻辑需要审计，不能只迁移 ROS 层。
- 可用“选一个实际可获得的来源”或有证据的相关/保守摘要策略；不得固定只收较小 ID 报告者而在仅大 ID 可达时丢失全部边。未知独立性策略必须显式，不能仅用降采样冒充独立性证明。
- `use_aa=false` 时仍解析、审计 AA，不更新 A–A 信息，不把已禁用 AA 造成的拒绝误报为 TA 融合故障。OFF 模式记录新接口输入但不加 UWB 因子。

### 4.4 去重与生命周期

观测身份至少包含：

```text
(network_instance, driver_session_id, reporter_id,
 source_session_id, source_frame_seq, kind, node_a, node_b)
```

`batch_seq`、发送轮次和 shard 用于追踪交付，不替代上述物理报告身份。同源帧不同 peer 必须都可接受；跨轮重发同源帧同 peer 不产生新信息；同距离的新源帧仍是新报告，其是否独立由上一节策略决定。

内部可扩展原身份结构或用无碰撞映射，不得截断 64 位会话、用未经碰撞处理的 hash 替代身份，或仅以回调到达次序分配“新测量”。正确处理 32 位源序号回绕、乱序、driver 重启、source 重启和旧会话迟到；多个 source 不因彼此到达而互相退休。

所有待处理、去重、退休和映射状态均有容量与退休规则。缓存过期或容量淘汰后旧报告不能重新成为可融合新观测；达到无法保留必要证据的容量时拒绝并诊断。不得为通过测试缩小到只测无乱序输入。

## 5. 仿真与运行链迁移

### 5.1 仿真数据

- 从现有物理测距生成新消息，保留独立随机流、测距发生时刻、真实 3D 量程/LOS 和合法传感器外参。不得从坐标/轨迹真值构造额外估计器先验。
- 一条源帧的边共享真实声明的源帧身份。不同采样时刻的旧距离不能为凑 batch 被赋予同一个新源时间；接口等价场景允许单条 batch，同时必须另有多 peer、跨 shard 的契约测试。
- 本地 reporter/node_a/receiver 为 N0；远端 reporter 为报告该固定边的逻辑节点。远端是报告转发，不意味着对该边重新测距。
- 仿真已知采样时刻可以在明确的模拟时钟映射下填写 `source_frame_stamp`，以有效标志和运行元数据证明；精确误差为 0 只限确有该性质的合成配置。RSSI 未模拟则 NaN/false；hold age 未模拟则无效/65535，模拟了则由实际事件计算。
- 只有确实为每条边生成新的独立测量样本时才可在合成场景置 freshness=true，并注明软件证据。驱动语义场景必须覆盖 freshness=false、未知时间及 NaN；不通过填 true 掩盖拒绝路径。
- 新接口回归保持当前 TA/AA 测量调度和物理噪声。额外的 50 Hz 本地帧/5 Hz 远端完整轮次场景验证回调吞吐与分包，不宣称无线或真实 TWR 频率；转发同一源帧不会刷新测量身份。
- float32 转换误差单独核对。真实设备毫米编码属于驱动链路；若另加毫米量化仿真，应单列与接口精度变化的区别，不偷偷改旧物理基线。

### 5.2 录包、回放、故障注入与可视化

- 更新 simulator capture、observer、replay、delivery schedule/analysis、raw identity、evaluate/compare、visualize、启动 readiness 和 manifest。实际字段使用需逐项检索，不止替换 topic 字符串。
- 默认新包只含统一 UWB topic。回放时不同时播放旧 observations 和通过 bridge 再生成的 observations，也不与 live simulator 双发。
- hash 覆盖 batch 元数据及全部单条语义字段，明确 NaN 的稳定表示；分别记录 batch 数、条目数、唯一报告数及融合数。不能继续拿“一条 ROS 消息”代表“一条边”。
- 传输故障区分整 batch 丢失/重复/延迟与单条观测过滤。单 anchor 失联只移除相关条目，保留同 batch 其他边；剩余条目的身份和源时间不变。
- NLOS 距离扰动只改指定 `distance_m`，原始值、偏置和注入依据写测试日志。不向不存在的 `variance_m2/propagation_status` 赋值，不因为测试知道 NLOS 就给生产估计器泄漏标签。
- 延迟/重放原 observations 时保留历史 `received_stamp` 和 batch Header；另记录此次交付时间。若模拟驱动首次接收，按事件实际生成接收时间，并在场景名称与日志中区分。
- 历史旧 bag 不覆写。最小交付允许明确识别并拒绝旧类型，提示使用历史版本；如做显式离线转换，必须生成新数据集、保存旧新身份映射及假设，不伪造 RSSI/时钟/独立性证据。旧类型包可仅为历史解码保留，不再是生产默认依赖。
- 旧/新 schema、hash 算法版本和转换关系明确记录，不能要求不同消息格式的字节 hash 相等。等价性比较以端点映射后的物理报告和时间/距离/权重为准。

### 5.3 构建与存储

编写时 build/test/run 脚本仍包含项目内 build/install/log 和安装二进制定位。实施须遵守主机规则：先 `data-run --check`，所有构建、测试、录包和分析经 `data-run -- ...`，产物放 `/home/dev/ws_iap`。将有效路径配置贯穿脚本、子进程、binary identity、日志和测试临时目录；不得简单 source underlay 后又由 `env -i` 丢失依赖。

使用一个明确的消息 underlay 和 LIUO install，防止加载旧版本/重复同名包。不迁移或删除当前输入来腾路径，不通过绕过 launcher、系统盘 fallback 或额外 worktree 解决路径问题。每次运行沿用唯一 `LIUO_RUN_DIR`、owner manifest、超时和磁盘保护。

## 6. 实施切片与主要入口

| 切片 | 主要工作 | 完成证据 |
|---|---|---|
| M0 基线与构建 | 核查源码/接口摘要；登记 ID、时间、噪声配置；消息依赖和数据盘构建入口 | 干净环境可生成/导入接口；没有重复包 |
| M1 统一回调 | 新 batch 订阅；逐条适配、校验、复合身份、拒绝日志；图修改仍由 owner 完成 | 合法 TA/AA 可进入现有处理；异常不污染图 |
| M2 仿真生产路径 | 新消息发布/capture、显式 ID 映射和仿真依据 | 真正 simulator→生产回调闭环，无旧 topic 中转 |
| M3 运行工具 | observer、replay、hash、故障注入、可视化、评估和 schema 版本 | 新包录制/回放一致；历史包处理明确 |
| M4 验证交付 | 契约测试、ROS 集成、负载、代表性图回归、命令与报告 | 第 7 节全部必需项有证据 |

入口：`packages/liuo_tunnel/apps/liuo_tunnel_node.cpp`、`include/liuo_tunnel/uwb/measurement_queue.hpp`（相对 `packages/liuo_tunnel`）、`packages/liuo_tunnel_sim/src/rail_sim.cpp`、两包 CMake/package.xml、`packages/liuo_tunnel/config/`、`tools/{build.sh,test.sh,run.sh,run.py,observe.py,replay.py,raw_identity.py,delivery_schedule.py,delivery_analysis.py,visualize.py,evaluate.py,compare_runs.py}`。同步搜索 launch、测试、地图导出 provenance 和辅助工具中的旧合同引用；历史结果文档不批量改写。

主测试接缝是“公开新 UWB topic → 实际 LIUO 回调 → 决策日志/队列消费/图统计”；少量纯函数测试补身份与时间边界。不要只测独立演示订阅者或内部 getter。

## 7. 测试矩阵与验收

先冻结合成输入、配置、预期报告身份集合及最小路线，再运行测试。下面是待执行要求，不是现有通过记录。

| ID | 必测场景 | 可观察的通过条件 |
|---|---|---|
| T01 | 干净依赖构建、C++/Python 类型、启动/QoS/remap | 同一接口包，真实 publisher/subscriber 匹配，默认 topic 类型正确，无旧 topic 生产依赖 |
| T02 | 多 peer 同帧、多 shard、缺尾 shard、本地与远端交错 | 每条新边恰好交付一次；不等齐；本地流不被远端节流 |
| T03 | ID 映射、N0 与旧 anchor 0、未知节点、自环/reporter 错误 | N1 正确对应内部 anchor 0；非法项拒绝，合法同批项继续；地图 ID 不变 |
| T04 | NaN/Inf/非正距离、非法枚举、空/超大 batch、无效 shard、未知 RSSI | 分类可审计，不崩溃；未知可选质量不被冒充有效值；资源有界 |
| T05 | 两种 mapped、unknown/receive-only、零/非法时间、NaN 时间误差、hold sentinel | 依第 4.2 节接受/拒绝；绝不用 Header/received/now 偷换源时间；不把非法 nanosec 悄悄归一化 |
| T06 | 同帧多边、同帧跨轮重发、相同距离新帧、32 位回绕、乱序、source/driver 重启 | 复合身份不冲突；新会话合法数据接受；旧会话迟到及退休后重复不重新入图 |
| T07 | 方差模型错误/缺失、freshness=false、双向 AA、只收到大 ID 一端 | 模型来源可追踪；无独立性依据不加独立信息；双向副本不加倍；来源选择不硬编码小 ID |
| T08 | `use_aa=false`、UWB OFF、正常 AA summary/自标定/复用路径 | 开关有效；AA 不生成车辆距离因子；回调不直接改图；地图吸收信息不重复加入 |
| T09 | 新包录制→原样回放→observer/估计器 | 完整语义 hash 和身份集合一致，未知状态不丢失；batch/观测/融合计数分清；旧包明确处理 |
| T10 | batch 延迟/重复/乱序/缺失、批内单 anchor 过滤、NLOS 偏置、ROS 时钟回退 | 原身份/源时间保留；只影响指定条目；过期/未来队列/重置行为正确；无旧字段依赖 |
| T11 | 模拟驱动语义输入→实际生产回调 | 覆盖 NaN/false/unknown 的 driver-like 合成输出；拒绝原因符合合同；至少一组有明确模型依据的 TA/AA 实际入图 |
| T12 | 60 s 软件负载、慢订阅、缓存上限/退休 | 使用合成 50 Hz 本地流与 5 Hz 远端轮次，逐边/10 s 子窗口计数；队列/内存有界，不把新报告率当独立测距率 |
| T13 | 代表性真实仿真与迁移前等价性 | 在新接口实际完成初始化并持续输出；物理距离/时间/端点/噪声对应，差异仅限已声明 float32/策略变化；无未解释丢边或因子翻倍 |
| T14 | 自标定→导出→独立复用的受影响路径 | 至少一条能完成初始化和地图使用的代表性闭环；ID/来源/相关信息正确，不能只有 load JSON 成功 |

T11 不要求连接真实串口设备。可复用驱动已有 PTY/消息夹具，也可直接发布按当前 `.msg` 构造的合成观测，但必须送入生产 LIUO 节点并标记来源；另写的 subscriber 不算接入测试。

T13/T14 复用现有阶段三代表性配置与路线，先跑最小有效片段；若缩短后不能支撑初始化/建图，则使用已有 360 s 代表性入口所需时长。可合并两项的运行证据，不重做阶段三全部对照和全故障矩阵。对比迁移前基线必须记录原始测距身份对应和实际语义，不用“同 seed”替代同输入。

GPU 预检及正式 GPU 运行单列；CPU 可以验证组件与诊断，不能替代 GPU 回归。有硬件/环境障碍时继续独立接口与测试工作，最终明确未执行项，不把缺少必要仿真融合证据写成软件全部通过。

完成门槛是合同正确、运行链可用、合格输入实际融合、回归无未解释退化及证据可复现。位置/高度/姿态误差与状态、初始化耗时、延迟和资源如实报告；不新增定位精度硬门槛，也不借本次迁移修饰已有弱高度、失败或过度自信问题。

## 8. 交付与非目标

交付：

1. 统一接口实现、仿真生成、配置、构建/启动依赖和受影响工具。
2. 有意义的自动化测试、实际生产回调集成和代表性仿真证据。
3. `docs/spec/uwb_observation_input_contract.md`：最终 ID、时间、噪声/相关性、去重、QoS、回放及旧包策略。
4. `docs/uwb_observation_unification_results.md`：每项测试通过/失败/未执行、源码和接口版本、完整命令、数据盘 run 路径/摘要、已知限制。
5. README 当前入口；本地按检查过的逻辑变更提交。正式 live 证据绑定干净已提交版本，不 push、不改历史，不删除用户工作。

本轮不改无线协议/MCU/射频固件、不要求实机频率/同步标定、不引入 NLOS 分类器、不改地图算法和物理部署、不新增规划控制、不全面治理高度/初始化历史问题。仅修复阻碍统一接口主线的实际依赖；真实部署缺少的标定单列，不据此停止可执行的软件开发。

仅完成全部必需交付与验收后，后续开发 Goal 才可 complete。单次失败应修复或调整有依据的方案并继续；不能在写完计划、生成文件、编译通过或话题可见时提前结束。
