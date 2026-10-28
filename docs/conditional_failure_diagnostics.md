# 正常车辆固定失败片段：条件诊断报告

状态：B实施中；S/L-D2已完成执行与初步审计，D1待完成，D3已提取冻结片段指标但完整归因未完成。范围及执行前计划见[当前Goal修订](spec/engineering_diagnostic_scope_20261027.md)。禁止把本报告当工程定位成功或正常汽车全程有效性认证。

## 冻结观测与比较规范

S原包190229（SHA256 `8de5439e8e35ae747ee42842aa505b8ec5290548a70b4386b47c71df82ccdd7a`），关注ROS[180,320]秒；L原包134947（`05095c59adad36e908bfb802978e1ecc3ec3b63686755a03c081cb5d40b13005`），关注[45,120]秒。两者保留相同原包前缀启动，不真值补状态，不换更容易片段。D1固定车辆真值是乐观条件图；D2只固定Anchor真值、估计车辆，不真值去畸变。D3无真值输入。

D3固定首输出yaw/translation，只用首输出确定规范，不在关注片段重新对齐；D2固定世界坐标直接评价，不作真值起点对齐。原始带噪TA及实际交付AA的精确身份集合仍待统一提取核对；当前只已核对输入包身份，不能声称三图有效信息完全一致。AA在D2无车辆信息。

共同原始集合现已提取到`/home/dev/ws_iap/conditional_diagnostics_20261027/{S,L}_observations.json`，只读取冻结原包交付topic，不读取真值或全局物理AA。S320前缀22873 TA/1190 AA，measurement_set_sha256=`c8d332df4c8de81efc5a0f3154267121b51be0a17fd73b7c93ee939a16daa822`；L120前缀6552 TA/234 AA，哈希=`d45feb6ff211192b3c5b46fde53b0bc99aede5a6991bba9973fbe2e553b4a49c`。本输入没有重复交付；保留交付身份及canonical物理样本数值，同ID冲突会失败。该集合代表可用输入，D3实际使用的窗口子集与D2接纳集仍待交叉核对，不能称有效信息完全一致。

命令：在ROS Jazzy及原共享消息install环境中，经`data-run --`运行`python3 tools/extract_conditional_observations.py --run <D3-run> --seconds <320或120> --output <上述对应路径>`；S D3为222403，L为224427。完整原包SHA重新读取验证，输出拒绝覆盖。

D2入口232238/232504均在3.005秒因必需配置字段缺失退出，保留为无效诊断；未提供任何定位结论。前者缺零数值初值，后者缺已知地图启动收集时长；修正见范围清单，传感器/噪声/因子未调。当前232558绑定db02f0d、实际节点bf79d67，使用声明零初值与已有8秒收集时长，已进入持续输出，尚待完成和评价。

## D3现有实际联合证据

假设：既有实际联合运行确实在冻结片段出现扩图/漂移问题。固定输入与唯一改动：仅离线切片评价既有不可回写的在线输出，估计器与输入未变。支持方式：固定规范误差与同期候选/接纳事件对应；不能仅由这些误差判断信息不足。

| 场景/实际运行 | 输出/关注段 | 固定规范位置RMSE/最大误差m | 沿线/横向/高度RMSE m | 姿态deg/速度m/s |
|---|---|---|---|---|
| S 222403（source ff12ad5，算法7b58726） | 1401条，180–320秒 | 6.842311 / 25.155377 | 5.569422 / 1.681508 / 3.601569 | 0.384681 / 0.365513 |
| L 224427（source eb29842，算法7b58726） | 751条，45–120秒 | 9.930578 / 17.674808 | 4.299385 / 8.919269 / 0.760467 | 0.681477 / 0.344375 |

S的A2候选在190秒发生快照过期，192秒后批次仍分支分离；A3候选也分离，A1最后实际接纳TA269.6秒，随后轴向误差增长。L实际A3/A16初始化，A7分支拒绝并有横漂。以上是事件和误差的相关性，不是根因证明。

补充S200只观察运行225028/225744覆盖A2前半段，不覆盖S完整关注段。225744的32个完成候选有27个阻尼到顶；1mm/0.1mm、固定其他联合变量的声明方向试探未找到下降。有限步长/方向不证明联合驻点，不能把停止判据或局部Hessian当全局可辨识性。此结果不支持无依据继续微调求解器。

补完整日志的S320重放234440（启动011acab、节点1c7f413）保持同原输入/配置/权重和固定窗口，输入语义hash与S-D2逐topic相同；新增日志仅序列化候选/测得几何。它没有逐位复现旧失败：A0仍12.4秒初始化，A1从旧96.8变96.9秒，新A2在191.8、新A3在283.4秒接纳；旧222403在本片段只有A0/A1。新关注段1401输出RMSE1.654355m，沿/横/高0.044702/1.641188/0.203452m，最大2.408368m，仍有显著横漂。旧6.842311m及轴向漂移不删除、不改判，不据新结果声称修复成功。数值/调度如何改变分支尚不能区分，不能把日志当改善性能的因果干预。

后续D1使用234440保存的实际局部观测/初值，必须标明它与旧失败222403的估计姿态条件不同；原始TA/AA集合相同，不等于两次派生几何/实际接纳子集相同。32种独立候选和全前缀条件图不能回填旧失败未记录的初值，也不能证明旧失败只由信息不足造成。L同样只补原120秒前缀的日志，禁止再换更容易的片段。

复现命令（项目根，输出留在原run的export/analysis；`focus`独立文件不覆盖历史报告）：

```bash
data-run -- python3 tools/analyze_road_diagnostic.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T222403Z_836 --truth-run /home/dev/ws_iap/.runtime/0/liuo/20261027T190229Z_933 --truth-format straight_csv --focus-start 180 --focus-end 320
data-run -- python3 tools/analyze_road_diagnostic.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T224427Z_941 --truth-run /home/dev/ws_iap/.runtime/0/liuo/20261027T134947Z_968 --focus-start 45 --focus-end 120
```

JSON及世界/沿横向图分别为`road_fixed_start_diagnostic_focus_180_320.*`和`road_fixed_start_diagnostic_focus_45_120.*`；JSON顶层保留原输出范围统计，`focus`对象才是本次诊断段，图仅画关注段。未用完整路线新运行替代片段解释。

## S-D2：固定真实Anchor，车辆仍由原传感器估计

D1首轮独立求解已执行，产物`conditional_diagnostics_20261027/S_D1_234440`及`L_D1_235022`。L固定真值车辆轨迹后出现A7分离3.474308m、原总代价差0.095421的候选，以及A14分离17.136852m/代价差0.000054，有限搜索不能证明找全解；尚需独立可行下降及真值仅评价检查。S的SLSQP虽32次返回success，但额外独立1mm/0.1mm原代价试探中30个有下降，部分高代价候选下降达9511，不能据返回码称数值收敛；最佳候选也有0.000358下降，当前无充分唯一性结论。

仅对S条件诊断做一次数值区分实验：同全部原始样本、同patch/32初值/约束和300迭代上限，只将优化目标及梯度统一除以残差数，ftol同除该数，使原代价绝对停止容差仍1e-9；报告的cost/信息矩阵仍为原尺度，不改变噪声或相对权重。假设为SLSQP大数值尺度影响停止/线搜索；预期有效候选不再保留明显可行下降。若无支持不沿此继续微调，不将其解释成物理信息不足。原始首轮及probe文件保留。

232558（源db02f0d、节点bf79d67）完成320秒原前缀，3200云/64000IMU/24063UWB/64000clock，墙钟319.995319秒，3120输出ROS9.0–320.9。关注[180,320]有1401输出，直接世界规范RMSE0.296887m、最大0.773029m，沿/横/高0.011990/0.149557/0.256185m，姿态0.179186deg、速度0.016621m/s。1400 DEGRADED/1 LOST均保留；不能因误差较小将状态改判正常。

条件审计确认22873条TA全部对应冻结输入决策，前缀22749 ACCEPT_TA/117 WINDOW_EXPIRED/7 SHUTDOWN_PENDING；关注段10763 ACCEPT_TA，记录决策状态的残差RMSE0.101440m。1190原AA保留交付但全部AA_ABLATION_DISABLED；图中A变量/AA因子/地图prior/安装因子均为0，Anchor是exact常量，没有导出地图。bias分量范围、状态、输入集合哈希见`export/analysis/conditional_vehicle_audit.json`。首次审计将CSV十进制double直接与原float32比较，出现最大5.68e-14的打印差异；改为恢复原wire float32后精确比较，无范围容差放宽。

相比同片段D3的6.842311m RMSE，本次理想Anchor条件未出现同量级漂移。这只排除了“在本次条件/初始化下剩余车辆链路仍必然同量级失控”的解释，不排除运动补偿近似、噪声或联合耦合，不证明未知Anchor问题可辨识。D2采用固定地图入口8秒收集/零数值初值，D3采用未知地图规范初始化；此差异明确保留。

复现：`data-run -- bash tools/conditional_road_estimator.sh --scene straight --source-run /home/dev/ws_iap/.runtime/0/liuo/20261027T190229Z_933 --config packages/liuo_tunnel/config/straight_local_static.json --map /home/dev/ws_iap/conditional_diagnostics_20261027/S_oracle_anchors.json --seconds 320`。独立评价：`data-run -- python3 tools/analyze_road_diagnostic.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T232558Z_196 --truth-run /home/dev/ws_iap/.runtime/0/liuo/20261027T190229Z_933 --truth-format straight_csv --oracle-world --focus-start 180 --focus-end 320`。审计：`data-run -- python3 tools/audit_conditional_vehicle.py --run /home/dev/ws_iap/.runtime/0/liuo/20261027T232558Z_196 --observations /home/dev/ws_iap/conditional_diagnostics_20261027/S_observations.json --start 180 --end 320`。图/JSON位于该run的export/analysis/conditional_oracle_vehicle_focus_180_320.*及_route.png。

## L-D2：原8秒启动失败，30秒收集取得有效关注段

234156绑定d0ceddf、节点1c7f413，使用事先声明的原Cholesky/30秒收集/零初值；120秒原包前缀，1200云/24001IMU/6786UWB/30787clock，墙钟120.001624秒，ROS31.0–120.9共900输出。关注[45,120]的751输出全为DEGRADED；直接世界坐标位置RMSE2.717660m、最大4.095605m，沿/横/高0.020714/0.604090/2.649589m，姿态0.179540deg、速度0.054584m/s。原D3同段横向RMSE8.919269m；D2横向较小，但高度仍有数米误差，不能称理想Anchor条件下车辆链路已经完全正常。

全部6552条TA在决策流有对应原始样本；关注段4400 ACCEPT_TA，记录残差RMSE0.103495m。原始234条AA全部无车辆因子，未知Anchor状态/地图prior/安装因子为0，无导出地图。bias范围及审计见`234156/export/analysis/conditional_vehicle_audit.json`；世界/沿横高图与指标为`conditional_oracle_vehicle_focus_45_120.*`及_route.png。不能由小TA残差排除高度分支或弱约束；也不能只凭前述线性系统停止判为不可辨识。30秒收集与原8秒的差异说明初始化上下文影响此次可运行性，但未独立证明具体数值或物理根因。

复现入口同S-D2命令结构，source-run改为134947，config为`packages/liuo_tunnel/config/road_conditional_anchor_bootstrap30.json`，map为`L_oracle_anchors.json`，seconds120，省略scene straight。评价使用`--oracle-world --focus-start 45 --focus-end 120`及road_npz默认；审计使用L_observations.json和start45/end120。生产估计器没有车辆真值输入；这些oracle结果不进入A的定位闭环结论。

## 归因表（当前证据边界）

### D1可复核结果与数值局限

求解器不读取Anchor真值；独立`evaluate_conditional_anchors.py`在求解后才读取oracle坐标，只评价和画图，不按误差选分支、不回写report或生产数据。所有“best”均由求解时原物理cost选择。S/L报告分别在数据盘`conditional_diagnostics_20261027/S_D1_234440_normalized`及`L_D1_235022`；原S未归一化失败保留在`S_D1_234440`。S归一化只改变独立SLSQP数值尺度，32个相同初值的代价从原最高约8.38e7收敛到[12268.412,12310.610]；无生产求解器修改、无噪声/相对权重变化。

| 条件集合 | 最低原cost/近代价候选 | 代表性分离（不以真值挑选） | 信息与残差解释 |
|---|---|---|---|
| S：22873 TA/1190 AA；A0/1/2/3各一实际测得patch | 12268.411714；11个cost差≤2的有限候选 | start24与17：A2分离2.166989m、A3分离2.318908m，总代价差0.526973 | A2/A3 TA白化RMSE0.985950/1.013090；固定其他Anchor的局部信息最小特征值10.3259/12.7113仍不能排除分支 |
| L：6552 TA/234 AA；A11/3/16/7/1各一实际patch | 3426.007860；8个cost差≤2候选 | start3与7：A7分离3.474308m，cost差0.095421；与6：A14分离17.136852m，cost差0.000054 | A7 TA白化RMSE1.009871，局部信息最小特征值48.9717仍有显著分支；A14只有184 TA，局部最小特征值0.002999，明显弱 |

S的A6在此前缀没有TA，只有AA，完整条件信息有两个近零方向，未来站点候选可相差数百米；这是缺少直接观测的预期限制，不能拿它冒充A2失败原因。上述表特意单列实际扩图相关A2/A3/A7，而不是只展示最弱未见站。L完整信息最小特征值约0.000438，完整谱和弱向量、逐站/逐边cost均保留在候选JSON。

S的原始梯度范数约11808，与活动几何边界相关，不直接作为不收敛证据。独立可行方向审计最初错误地把最终点容差1e-7m用于试探域，允许约3e-8m越界后“降价”。解析斜边界反例已复现并回归通过，修正为仅1e-12m算术余量；未改候选/物理域。旧审计及truth_evaluation.json保留，当前S应引用`S_D1_234440_normalized_strict_probes/feasible_probes.json`及`truth_evaluation_strict.json`，11个近代价候选均未发现cost下降超过1e-6；有限1mm/0.1mm方向不能证明完整KKT或全球唯一。L的8个近代价解最小域余量至少0.045m，旧试探不触边，且均无超过阈值下降；不重复其求解。独立数值缺陷只属于检查器，不能推成生产漂移根因。

图：S `truth_evaluation_strict_candidates.png`，L `conditional_anchor_candidates.png`。红叉仅为后验评价；S best的A2误差0.0232m，L best的A7误差0.1080m，但这些真值误差不得成为选择“正确”分支的依据。近代价分支证据说明现有观测集合对这些Anchor的全局分支区分有限；不是任意场景的不可辨识定理，也不排除更低未搜索解。

D1命令：`data-run -- env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 timeout 600 python3 tools/solve_conditional_anchors.py --observations <S或L_observations.json> --joint-run <234440或235022完整run路径> --truth-format <straight_csv或road_npz> --output <新目录>`；S当前结果另带`--normalize-objective`。只审计已有候选用`--audit-existing <原结果目录> --output <独立probe目录>`，不会重解或覆盖。评价命令为`data-run -- python3 tools/evaluate_conditional_anchors.py --result <结果目录> --anchor-reference <S或L_oracle_anchors.json> --probes <probe目录>`，S修正审计另带`--output-label truth_evaluation_strict`。输出拒绝覆盖原评价。

### D3实际子集、移交和运行差异

新S234440有10批候选、L235022有13批。`candidate_inputs.jsonl`记录每批初值、局部初值、实际TA源身份/值/时间插值和静态AA充分统计；`candidate_input_audit.json`逐条对照冻结原包，TA全部匹配，最大最旧TA/patch年龄分别4.996/5.0秒。每批AA身份由接收顺序的pending前缀重建并核对count、mean（1e-9m）、weight、scatter（1e-7）一致，明确为重建而非直接逐因子叶导出；实际完整图还包含已有联合因子与边缘信息，不能把这些新增列表称整个图。

S最终774新AA/3边、416 pending，320.0秒地图773精确叶；L最终94 AA/1边、140 pending，120.0地图94叶。首融合96.9/80.1秒仍接纳1.012秒旧AA，证明AA不依赖5秒候选窗。已知/未知端点及相关prior由现有静态审计检查，未把pending当已融合。新L关注段RMSE9.756465m、沿/横/高4.202165/8.773205/0.749194m，继续出现同类横漂；原224427的9.930578m不替换。

完成候选S38个（33阻尼到顶）、L73个（72到顶）；S/L各有2/6批相对分支仍分离。逐候选梯度、有限可行试探和终止标志见candidate_branch_diagnostic.json，不用停止标志证明物理唯一。S/L记录最大101车辆状态、845/754因子、RSS346.582/385.219MiB，最大优化79.187/57.274ms；输入hash逐topic与各自D2完全相同。完整原始及有效子集、旧窗口信息移交相关测试均可追溯，但新S没有逐位复现旧失败，数值/调度与候选接受的贡献仍未区分，不为此继续性能调参。

L-D2首次233329（4f39421，只观察日志节点）在ROS9.1秒、首输出之前抛出x0附近IndeterminantLinearSystemException，无有效关注段输出。它使用原Cholesky分解；前8秒真实交付TA包括A11/A3（161/160条），不能因异常就称没有Anchor信息。保留原失败。

下一最小区分实验预声明：假设是启动线性化的数值分解敏感性，而非由本次异常直接证明物理不可辨识；固定相同120秒原前缀、同oracle地图/零初值/8秒收集/噪声/因子，只通过`road_conditional_anchor_qr.json`将odometry.factorization从默认CHOLESKY改为现有QR。预期观察是否仍在同启动移交处失败；若QR可运行，仅支持分解敏感性的证据，不证明信息唯一或原实现bug，后续必须保留此条件差异。若同样失败，停止沿分解设置微调，改查启动图秩/规范；不通过增加真值位姿先验绕过。该配置与road_local_static_diagnostic的唯一差异应为factorization，执行前精确核对。

QR实验233509（392018e）在12.7秒也出现不定系统，未达到关注段，停止该分解方向，不能据此排除所有数值问题或证明信息不足。新假设仅检查启动上下文：原始TA前缀A11从1.0秒、A3从1.004秒、第三个A16从27.36秒开始交付，不读取真实车辆/Anchor几何来挑初值。下一次固定同原输入和零初值，恢复原Cholesky基线，只把已有bootstrap_s由8延至30秒，使启动图实际收到第三站；配置road_conditional_anchor_bootstrap30与原仅此字段不同，派生器保留显式值。预期是检验是否可进入[45,120]段，若仍失败不继续逐秒微调，需要具体图结构证据。没有扩大冻结窗口、没有增加测量或真值位姿先验；此初始化收集差异必须报告，不能归为原方法性能改善。

233851在首次求解前被owner原30秒无输出保护终止，未检验到上述假设。最小反例：bootstrap_s=30要求收完30秒数据后求解，owner同时要求30秒之前已有里程计。只修独立D2入口首次输出deadline为max(30, bootstrap_s+5)且bootstrap_s限定(0,60]；首输出后仍30秒无进展保护，整个run总超时seconds+30、250ms发布deadline、磁盘/子进程保护不变。8秒原入口仍30秒，故S已完成数据不受影响。回归覆盖8/30/非法61秒；原失败保留，下一次仅修owner等待，不改估计器或物理条件。

| 类别 | 当前结论 | 可复核依据及限制 |
|---|---|---|
| 已证实信息限制 | 在声明的固定真值车辆/原噪声/合法测得patch条件图中，S的A2/A3及L的A7存在显著分离的近代价有限解；L的A14弱、S的A6无TA | 上述原cost差、原始集合hash、候选坐标/谱/有限可行试探；是观测分支区分有限的证据，不是全球唯一性反证定理或实际联合后验 |
| 已证实实现缺陷 | 历史有限域线性试探在非线性retract后越界、入口地面支持域不一致、不同联合规范下直接比较世界候选、边缘先验重线性化问题已有反例与修复 | 见原results及相应测试；这些历史缺陷不自动等于当前漂移根因，不改判历史失败 |
| 已排除因素（限定范围） | 所引用D3未漏掉声明原始topic/字段/计数；D2/D3原输入hash相同；AA交付账本和真实融合可追溯；S理想Anchor条件没有同量级轴向漂移 | 不排除运动补偿近似、车辆模型、数值/调度或联合耦合；不能把S结论推广到L高度 |
| 尚未确定因素 | 原S失败与新S分支不同的数值/调度原因；L理想Anchor仍有高度偏差的分支/弱信息/车辆链路贡献；条件图与联合边缘化的定量差别 | 最小后续：只冻结旧失败附近一个候选的真实图/值/事件顺序作确定性重放；对L只在同关注段比较高度分支的原始TA似然与IMU残差。不扩大路线或新增先验，不在本轮立即性能开发 |

本报告已有D1/D2/D3的条件差异、候选/子集、图和不确定性证据，不以退出码或局部满秩作结论。整体Goal仍未完成：A6真实生产静态压力/慢订阅恢复/容量/必要重置尚未验收；本报告的普通资源曲线不能替代A6。正常车辆全程定位有效性仍未验收。
