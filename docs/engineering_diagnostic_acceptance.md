# 修订 Goal：工程能力与固定片段诊断验收

验收范围仅为用户正式修订的A/B；[需求及C01–C16逐条映射](spec/engineering_diagnostic_scope_20261027.md)保持有效。历史完整S/L失败不改判，后移项目不标完成。以下是当前交付结论，历史开发记录中的“待完成”不覆盖此清单。

## A 工程能力

| 项 | 结论与证据 | 明确边界 |
|---|---|---|
| A1 静态身份 | 通过。StaticAnchorEdges反向/1000重传/内容冲突；StaticRecordInput精确覆盖与来源；test_static_replay两个驱动session/反向测试；002916累计rev1/2及旧版乱序重传；003902实际driver session变化后同210物理叶各交付两次 | 物理叶身份与容器record/version不同；合成物理来源可追溯，不认证任意硬件driver重启 |
| A2 缓存生命周期 | 通过。StaticEdgeGraph未知端点pending后真实ISAM2入图；S234440/L235022首融合96.9/80.1秒仍接纳1.012秒旧AA；003902图重建恢复105旧物理叶，之后重复交付零因子更新；4096未来TA在reset清空且入图0 | AA保持部署有效性，TA仍须车辆时间校验；未知端点有界等待不等于保证初始化 |
| A3 唯一计权 | 通过。原始因子与Gaussian summary实际cost/gradient/Hessian等价；真实ISAM2累计替换remove1/add1；已吸收前缀只融合新叶；002916满账本1077叶、8103交付后0新叶/显式因子、1prior；身份冲突及新ID容量越界各拒绝一次 | 独立高斯与来源证据是模型条件；不同ID不证明误差独立。不得从稠密prior假装撤销旧信息 |
| A4 地图恢复 | 通过。13项地图/规范测试、失败事务不提交、精确账本/相关信息往返；CAL230820地图原字节载入REUSE231612；003902两图各1prior和3显式边；异epoch记录拒绝，旧checkpoint新epoch构造拒绝 | 不捏造v1缺失账本；不自动迁移移动后的部署；软件clock reset和DDS恢复分别报告 |
| A5 rolling/snapshot | 通过声明接入合同。4项独立RollingMotion解析测试覆盖高速/yaw/lever/比力及缺失/乱序IMU拒绝；L235022/D2 234156原生rolling时间/ring/覆盖/单位外参与S234440/旧闭环snapshot非单位外参生产检查；实际补偿只用测得IMU及估计状态 | 不代表真实Mid-360或定位有效；不发布/输入真值去畸变。无需重跑7km |
| A6 资源/生产 | 通过声明有界工程工作负载。002916的85秒负载有63.59秒实际估计输出重叠；生产8103/8103、慢observer3781/8103，后段恢复无丢失；队列15/2048、生产RSS峰355.313MiB，固定账本/因子无重传增长。003902独立边界输入与图恢复审计通过 | 两次reset运行原评价失败与独立重评价FAIL均保留，见下文；不称单run全面PASS。DDS内部占用未直接测量，有限时长不证明任意长期无泄漏 |
| A7 旧路线闭环 | 通过。原360秒CAL031155→未知Anchor联合估计230820→相关地图/1077精确叶→独立360秒REUSE031819/231612。旧叶只在1个联合prior，1080独立新叶进入3显式边、0安装重复；002916补旧块生产重传 | 旧路线含三维激励且旧reporter LOS不完整。不是正常汽车/新S/L有效性认证；地图高度与未初始化站点不作成功声明 |

工程结论仅为：**静态 AA 信息生命周期、地图闭环、rolling 接入与声明资源边界得到验证。**

32项原工程定向测试及命令映射见范围清单。新增负载夹具、截段/seek真值边界、固定地图跨session坐标规范检查分别覆盖实际反例。生产节点最终仍为1c7f413：bf79d67后只增加候选初值和实际局部几何日志；不修改候选接受/权重。CAL/REUSE实际bf79d67与其数学一致；日志新增后已有S320/L120和A6实际资源证据。当前收尾变更仅工具/评价/文档，未改生产数学。只观察日志的候选求解统计S/L分别4158.776/3227.216ms（已完成候选总计），不是额外求解或隐藏观测。

## A6失败保留与验收依据

所有run前缀为`/home/dev/ws_iap/.runtime/0/liuo/`。

- `20261028T001651Z_732`：初次负载信息不变量已观察，但owner截段仍复制完整360秒truth，原run FAIL。新85秒002916修正后原合同及独立负载审计通过；不改旧报告。
- `20261028T003614Z_856`：70秒seek实际图恢复通过，但owner只给一个truth session，评价报错，原失败保留。
- `20261028T003902Z_728`：绑定e2f0603，owner修正为原35秒truth重复两会话，实际回放/图恢复再次通过；评价器把同一CAL地图datum错误地要求为每session独立datum，仍报错。解析反例及修复见`map_reset_datum_before.log`/`map_reset_datum_after.log`，3项回归通过。修复只用于评价，不按第二会话真值重对齐。
- 独立离线`003902/export/analysis/shared_map_datum_reassessment/report.json`仍为FAIL，且保留：其四项失败是通用“原70秒输入不变”检查与预声明“35秒重复两次＋UWB故障替换”不兼容。没有删除这些检查或改判该报告。既有通用报告不作为seek的接入验收器。
- 专用`boundary_input_audit.json`独立重读原包，**不调用replay/BoundaryFixture实现**：对700云/14000IMU/14000clock逐字段语义hash核对原35秒重复两次；解码生产实际收到的4458个UWB CDR，完整字段逐个比较独立重建的正常交付与12个声明负例。全部通过，并与observer及publisher摘要匹配。
- 专用`uwb_boundary_audit.json`保留原TA过期/未来/容量/旧driver断言，静态通道改由实际static图审计取代原raw ACCEPT_AA判据：4096未来TA清理且融合0、第二session156输出/2012TA融合；210物理AA每叶重复两次且内容不变，每个图105有效叶/3边/1prior。第二图首更新恢复105叶，随后0因子新增/移除。没有把已被丢弃的第一图作为仍存在的prior。

这些独立证据验收声明的工程重置机制；原运行/通用评价失败仍存在，不宣称定位有效或通用runner支持任意变换。完整计划、单一变化、命令与RSS见[负载计划与结果](spec/static_production_load_plan.md)。不再用重复回放来修饰通用报告。

复核环境/命令（源目录执行，所有产物仍在数据盘）：

```bash
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/liuo_observation/messages/install/local_setup.bash
source /home/dev/ws_iap/liuo_observation/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/audit_static_lifecycle_load.py /home/dev/ws_iap/.runtime/0/liuo/20261028T002916Z_591
PYTHONDONTWRITEBYTECODE=1 python3 tools/audit_boundary_input.py /home/dev/ws_iap/.runtime/0/liuo/20261028T003902Z_728
PYTHONDONTWRITEBYTECODE=1 python3 tools/audit_uwb_boundary.py /home/dev/ws_iap/.runtime/0/liuo/20261028T003902Z_728 --static-channel'
```

## B 固定失败片段

B完成的是有效条件诊断及可追溯比较，不是修到成功。完整计划、精确输入hash/有效子集、初值、坐标/噪声、图与命令见[条件诊断报告](conditional_failure_diagnostics.md)。关注片段始终S[180,320]、L[45,120]，必要原包前缀320/120秒不变；不换容易数据、不真值补生产初态。

| 场景 | D1固定真实车辆、未知Anchor | D2固定真实Anchor、估计车辆 | D3真实联合 |
|---|---|---|---|
| S | 32声明种子，11个近代价候选；A2/A3分离2.167/2.319m，cost差0.527；严格可行试探未反驳这些候选 | 232558关注段世界RMSE0.296887m，最大0.773029m，无车辆真值先验/去畸变 | 原222403关注段RMSE6.842311m、后续全程失控；补日志234440同原输入关注段1.654355m，初始化分支不同，旧失败不被替代 |
| L | 8个近代价候选，A7分离3.474m、cost差0.0954；A14弱方向与17.137m分支另列 | 234156世界RMSE2.717660m，其中高度2.649589m，仍有明显问题；原8秒Cholesky/QR失败保留，只延长原启动收集到30秒 | 原224427关注段9.930578m；补日志235022为9.756465m、横向8.773205m，继续同类漂移 |

D1仅为乐观条件图；局部满秩、有限候选搜索和有限方向试探均不证明全局唯一。Anchor真值只在求解后独立评价，不作初值/分支选择。D2中的纯AA不产生车辆因子；D1/D2的oracle产物不进入生产地图或工程定位闭环。D3新增日志记录实际初值、原始身份、窗口子集/移交、拒绝/停止与可行下降，不改变图。

归因：已有证据支持S A2/A3及L A7观测分支区分有限；L在理想Anchor条件下仍有高度误差，不能只归咎未知Anchor。旧有限域/规范/边缘化缺陷有独立反例，但不能自动解释当前漂移。原S失败与补日志运行的数值/调度差异、L高度分支/弱信息/车辆链路的相对贡献**尚不能区分**。最小后续仅为一个候选真实图/值/事件顺序的确定性重放，以及同L片段高度分支原TA似然/IMU残差比较；本轮不立即实施性能开发。

## 保留、版本与结论

原始S/L/CAL/独立REUSE输入、必要真值、地图、故障运行、候选JSON/seed/probe、报告及图均保留，不迁移或删除当前输入。诊断目录为`/home/dev/ws_iap/conditional_diagnostics_20261027`；A/B运行的manifest绑定各自干净源提交、实际二进制/依赖、配置及原包摘要，不能把旧运行冒称最后文档版本。无新全程FAST/LIO-only或多种子campaign；旧完整对照与失败见原结果记录。

可用性/小产物SHA清单：`/home/dev/ws_iap/engineering_diagnostic_delivery_20261028/inventory.json`，由`data-run -- python3 tools/inventory_engineering_diagnostics.py --output <新路径>`生成，拒绝覆盖。原始大包在此仅检查仍存在并列原capture摘要（各正式回放owner另已核对原字节），不冒称此清单重新hash全包。没有新录制重复原始包可退休；当前诊断中间候选与失败用于复核，继续保留。

**本 Goal 完成的是工程能力与可行性诊断；正常车辆场景的全程定位有效性尚未验收。**
