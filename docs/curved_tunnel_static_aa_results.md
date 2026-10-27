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

### S 旧路线生产回归：`20261027T152225Z_083`

绑定干净提交 `14fbd3a`，实际 RTX 4070 Ti SUPER，360 s 录包在 360.00085 s 发布完成。输入 72,000 IMU / 3,600 snapshot / 26,614 UWB batch 全部核对，原始内容 hash 检查通过。命令：

```bash
data-run -- tools/run.sh --formal --replay /home/dev/ws_iap/.runtime/0/liuo/20261027T031155Z_389 --config packages/liuo_tunnel/config/phase3_static_s_regression.json --installation packages/liuo_tunnel/config/installation/straight_simulated.json --use-aa --wall-timeout 480
```

run 路径 `/home/dev/ws_iap/.runtime/0/liuo/20261027T152225Z_083`；owner manifest 为 completed，`export/analysis/report.json` 单 run 合同 PASS，阶段验收仍 NOT_ASSESSED。实际未知 A 为 0/1/2，共接收 2,160 个独立静态叶，最终融合 720 个、两个显式 AA 因子，另外 1,440 叶因端点未初始化保留 pending。最后地图检查点在 t=360.0，含 718 叶；最终状态 t=360.9，比该检查点晚，因此不能把地图说成含全部 720 叶。其余 4 站未初始化；扩图日志保留未消除的位置分支。

首次输出 t=40.9（等待 39.9 s）。固定首轮规范的位置 RMSE 0.75249 m，分轴为 0.01760/0.02319/0.75193 m，高度弱误差未解决，未达到 0.30 m 参考值；输出 3197 个 DEGRADED 和 4 个 LOST。处理 p95 44.71 ms，端到端 p95 51.80 ms。该结果仅证明此旧路线上的生产静态接入与真实地图导出，不能替代完整 S/L、rolling 或独立复用。

另已确认旧 S 生成器只检查 AA 两端的量程/LOS，没有单独检查 reporter→车辆链路。旧包原样保留；本派生器只使用其中已送达的消息，但这不证明无线交付物理成立。新完整 S 采集必须补齐这项物理门限，旧回归不得作为 C10 的 LOS 通过证据。

独立复用的旧输入 `20261027T031819Z_304` 为 seed 1、启动 `[220,0.2,1.6,pi/2]`，与首轮 seed 0、`[200,0,1.2,0]` 不同。专用静态复用配置绑定其 raw SHA256 `5be10c3a8c89879865f0dc7c11d380d992ea429d51ddf6eca015c1e0a78764b4`；复用结果待实际运行，不按配置存在判通过。
