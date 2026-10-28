# A6静态通道有界生产验收计划

状态：85秒负载及负例验收通过；单独补查静态图的实际时钟重置。只补修订Goal的工程缺项，不是正常车辆精度或射频实验。

假设：已有地图精确吸收账本应使同一物理AA在反向报告、内容版本1/2重复及DDS发布者重建后始终不产生新信息；满账本不应因同块重传增长。慢observer的丢失与恢复不应被当作生产消费者丢失。

固定输入：独立旧REUSE `20261027T031819Z_304` 的前85秒，原MCAP不复制、不改字节、不改时间；`--replay-seconds 85`在独立预读中计算end-exclusive语义hash/count，生产回放与observer分别校验该前缀。加载旧CAL230820导出的原字节地图（SHA256 `89c3e21f1af3dc77d76b9ee372192d7fbea21ac74fd5447b43ae54d7b5eaafe8`）。唯一输入变化是静态软件交付负载：不把本轮raw AA转换为静态叶，改为从该CAL原始静态CDR审计中选取一条已吸收、早于2秒的真实物理叶，保留物理ID、值、方差、采集时刻、来源和部署epoch。

使用`static_lifecycle_load.json`：原REUSE车辆参数、噪声、TA/IMU/LiDAR合同保持，静态source指向被转发的原CAL来源；容量=地图实际1077叶。软件elapsed[2,83)以100Hz交付（不是100Hz物理测距），交替first/second与revision1/2，payload物理叶不变；25/45秒重建DDS publisher，记录未匹配时刻，不假称完整进程或物理部署重启。A7已经独立验证新估计器进程加载地图。独立best-effort observer depth37，在ROS stamp<51时每次回调sleep0.1秒，之后恢复。生产静态消费者仍为原可靠QoS100。

观测判据：传感器原始前缀hash/count匹配；所有收到的有效旧叶均对应已吸收身份；始终1个联合prior、0新AA显式因子/叶、0安装重加；满账本容量不增长。保存发送、生产接收、observer接收三条记录，逐交付时刻对照，分别报告损失和恢复。报告实际生产优化输出期间的负载时长（目标至少60秒）、RSS/队列/身份/因子数量及完整曲线；不以“进程没崩”替代这些检查。发布者重建窗口的未匹配不隐藏，不用observer代替生产消费者。

已实现夹具检查：8500个10ms轮次对应8100有效发布，两个publisher重建、方向/revision变化、8100唯一交付时间及原physical leaf字节不变；测试不替代真实生产。前缀工具只读原包并绑定全包SHA和前缀语义hash。所有执行经data-run，原GPU预检/1×250ms发布截止/总超时/无进展/磁盘/owned children保护保留。

计划命令：

```bash
data-run -- bash tools/run.sh --formal --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T031819Z_304 --replay-seconds 85 --config packages/liuo_tunnel/config/static_lifecycle_load.json --map /home/dev/ws_iap/.runtime/0/liuo/20261027T230820Z_728/export/map/calibrated_map.json --use-aa --static-lifecycle-fixture --wall-timeout 150
```

这组先验证重复压力、满容量重复、observer恢复及已吸收信息不增权；新增叶越容量的受控拒绝、部署epoch变化/必要重置仍须与既有单元/生产证据逐项核对，缺项才做最小负例，不把本组自动扩称全部A6通过。

## 首轮结果与受影响回归声明

首轮001651（源8b7a834）传感器前缀850云/17000IMU/5412UWB/17000clock全部匹配，8100静态交付全部到生产消费者、最大callback queue9/2048；慢observer收到3779、丢4321，55秒后丢失0。实际输出期间承受63.59秒负载，1077旧叶、1个联合prior，新增AA因子/叶/移除均为0。RSS生产最大349.133MiB，慢observer71.270MiB；原map字节未变。独立审计为INVARIANTS_OBSERVED_BUT_RUN_REPORT_FAILED，原run FAIL保留。

原FAIL来自新前缀入口仍复制完整360秒truth.csv，旧检查器据其末行要求输出到360.9秒；实际最后输出85.9秒已匹配最后包含的扫描。只修owner产物为截取到独立预读实际最后cloud stamp的truth行（按纳秒比较以保留85.90000000000001等表示），不改原数据、结束验收容差或旧报告。解析反例验证包含最后扫描、排除86秒及完整末尾且原source不变。必须新run验证该修正，不将001651改判PASS。

下一组仍固定同85秒原输入/原地图/车辆配置/满容量：用两条同端点、已吸收的原CAL叶验证累计版本。revision1携带第一叶，revision2携带两叶，再回到旧revision1；两条物理叶内容均不变，不能因版本增长或乱序重复增加权重。另加三个**明确负例包，不是新测量**：elapsed60改epoch，65保持原physical ID但range+0.25，70提交标记negative_capacity_probe的新ID。分别预期回调SOURCE_MISMATCH、owner事务IDENTITY_CONFLICT、BLOCK_CAPACITY；之后继续有效旧块直到83秒，图和账本不变。不是增加站点/改善几何/修改噪声来制造定位通过。前缀修复与这组负例各有独立反例/夹具检查，生产算法源码不变。

命令使用上方85秒命令并加`--static-negative-probes`。验收脚本`tools/audit_static_lifecycle_load.py`将发送、生产与慢observer逐交付时间对照、核对原物理值和累计覆盖，要求生产0丢失、后段observer恢复、每次图更新0新AA/0移除、一个prior、负例明确拒绝且随后可继续。DDS内部占用不直接暴露，只报告配置depth、应用队列与实际回调延迟/丢失，不能虚报内部队列测量。

## 85秒生产结果

`20261028T002916Z_591`绑定干净dd67be7，实际节点1c7f413。原运行合同PASS，独立`export/analysis/static_lifecycle_load_audit.json`为PASS_STATIC_LIFECYCLE_LOAD：8103发送/生产接收，生产丢失0；慢observer收到3781、丢4322，55秒后丢失0。实际估计输出期间负载63.59秒，DDS发布者重建2次，未匹配机会0。应用队列峰15/2048，最大回调延迟0.379918秒；DDS内部占用未测。生产RSS峰355.313MiB、末333.879MiB，慢observer峰70.867MiB。1077旧叶保持，0新叶/显式因子，始终1个联合prior；epoch错误在回调拒绝，物理内容冲突与新ID越容量各在图前拒绝1次，之后有效重传继续。

001651的原FAIL不变。驱动交付session变化/反向报告不改变物理身份的两个`test_static_replay.py`检查也通过；首次未加载消息overlay导致ImportError，补齐环境后通过，不改代码或断言。该测试与DDS publisher重建均不冒充整个ROS图的时钟回退验证。

## 静态图时钟回退补项（执行前冻结）

缺项：旧raw-UWB boundary已经验证TA队列在时钟回退清理，但不足以证明新增静态图在真实node reset后恢复。假设为同部署、同物理输入重复回放只重建被丢弃图；旧地图叶仍只在prior，本轮叶在每个新图中恰融合一次，驱动新session不增加物理身份。

固定原独立REUSE031819、CAL230820地图、phase3_static_s_reuse_regression配置，GPU1×；使用已有`--uwb-boundary-fixture`，70秒交付由原前35秒重复两次，第二次仅驱动session更新，传感器时间真实回退至原启动。唯一实验变化是既有边界交付变换：过期/远未来/4097条未来TA、35秒seek、新driver及旧driver迟到；无新噪声或物理样本。保留原变换清单并单列软件负例，不能称原样70秒物理轨迹。

预期：两次图session都持续输出；4096个未来TA被CLOCK_RESET清除且不融合，过期/旧driver拒绝；静态物理叶跨session相同，第二图恢复一份prior及本轮唯一AA，重置后无旧factor index残留/无双prior。既有raw boundary审计要求ACCEPT_AA，不适用于静态通道；新增审计必须核对static_edges实际图事件/账本与因子，而非删掉该项宣布通过。若失败保留，只诊断工程重置问题。

```bash
data-run -- bash tools/run.sh --formal --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T031819Z_304 --replay-seconds 70 --config packages/liuo_tunnel/config/phase3_static_s_reuse_regression.json --map /home/dev/ws_iap/.runtime/0/liuo/20261027T230820Z_728/export/map/calibrated_map.json --use-aa --uwb-boundary-fixture --wall-timeout 150
```
