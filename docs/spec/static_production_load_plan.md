# A6静态通道有界生产验收计划

状态：执行前冻结，当前尚未通过。只补修订Goal的工程缺项，不是正常车辆精度或射频实验。

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
