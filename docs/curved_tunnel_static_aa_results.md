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
