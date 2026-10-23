# 两个启动入口与显示验证（2026-10-23）

## 用户桌面复测后的修正

用户运行 `log/20261023T014951Z_092` 的 RViz 日志确认 OpenGL 4.6 正常启动，不能把此次空白归因于无图形服务。该次约 40 s 后被 Ctrl+C 中断，估计器仍在初始化；显示统计为 0 帧全局点云、0 个已初始化基站、40 次地图/状态更新、2,490 条接收测距。

检查发现此前 MarkerArray 配置错误地使用 `Marker Topic`，而本机 Jazzy `rviz_common/ros_topic_display.hpp` 的真实属性名是 `Topic`。因此初始化期间应显示的测距和状态文字未订阅；此前 YAML 结构检查及独立订阅检查未覆盖 RViz 的属性加载，不能作为窗口显示通过的证据。现已修正两处话题属性，显式启用根显示组，补齐视角操作工具，并在尚无位姿时显示清晰的 INITIALIZING 提示。初始化未解出前仍不伪造全局点云或基站。

回归覆盖 MarkerArray 的实际 Topic 属性路径及初始化提示，`log/visualization-marker-fix-tests.txt` 两项通过，`log/visualization-marker-fix-build.txt` 三包构建通过。本次不重跑未受影响的长程估计算法；修正后的桌面画面仍待有效图形会话确认。Ctrl+C 引起的 launch 非零退出是主动中断记录，与最初没有显示不是同一问题。

入口见 README 的「RViz 启动入口」。实现提交 `ec793ff`，launch 失败传播修正 `105cabb`。两者使用同一个新构建的 GPU 估计器；估计器数学未修改，只增加按订阅需求生成的估计点云及当前图内基站标记。新增显示进程不订阅真值、不向估计器反馈。

## 实际验证

| 检查 | 结果 | 证据 |
|---|---|---|
| 干净 underlay 独立三包构建 | PASS | `log/visualization-build.txt` |
| 外参、时间匹配拒绝、未知基站、去重体素、显示不修改位姿、时间重置 | PASS | `tools/test_visualize.py`、`log/visualization-unit-tests.txt` |
| 首次无图 launch，80 s、seed 0、GPU、无窗口 | PASS（入口及显示数据） | `log/20261023T014431Z_932/` |
| 加载上述首次地图，55 s、seed 1、独立起点/朝向、GPU、无窗口 | PASS（入口及显示数据） | `log/20261023T014619Z_181/` |
| 独立订阅四种显示话题 | PASS | `log/visualization-topic-audit.json` |
| 非法运行参数经 launch 返回非零退出 | PASS | `log/visualization-failure-propagation.txt` |
| RViz 实际图形渲染 | 未验证；离屏尝试失败 | `log/20261023T014416Z_958/runtime/rviz_console.log` |

首次输出 277 帧估计点云，80 次显示地图，最多 7,308 个体素、3 个基站，显示进程收到 5,088 条原始测距、画出 74 次可关联连线。复用输出 426 帧、55 次地图、最多 20,370 个体素、3 个基站，显示进程收到 3,497 条测距、画出 115 次连线。详细计数位于各 run 的 `runtime/visualization_summary.json`。原始观测记录由原有 observer 负责；显示订阅为 best effort，不用显示计数代替传感器完整性验收。

独立订阅器在首次运行的 40 s 窗口内收到全部 277 帧估计点云及基站数组、39 次地图及测距数组，检查 fixed frame 为 map、点数未超过 150,000，未发现错误。两个 run 的 manifest 都是 completed，保留 GPU、轨迹、状态及正常评估产物。初始化后状态仍是 DEGRADED，不因可视化而改成 TRACKING。本轮短程验证不替代此前完整路线证据。

复现（先 source ROS 与本项目 install，设置 LIUO_RUN_ROOT 和 ROS_LOG_DIR 到本项目 log）：

```bash
python3 tools/test_visualize.py
ros2 launch liuo_tunnel first_mapping.launch.py duration:=80 wall_timeout:=180 rviz:=false
ros2 launch liuo_tunnel reuse_map.launch.py \
  map:="$PWD/log/20261023T014431Z_932/export/map/calibrated_map.json" \
  duration:=55 wall_timeout:=150 rviz:=false
```

默认 `rviz:=true`。当前执行环境虽然设置了 DISPLAY=:0，但没有 X server；Qt offscreen 下 RViz/Ogre 无法建立 GLX 窗口并退出 -6。运行器将其记录为失败并收尾自己启动的进程，没有隐式无窗口回退。桌面窗口效果、交互操作仍需在有效图形会话验证。本轮另一次早期启动被干净工作树检查拒绝；继承的旧 LIUO_RUN_ROOT 指向 `/home/dev/ws_iap/.runtime/0/liuo/20261023T014355Z_489`，没有启动传感器或估计器；随后显式设置项目内日志路径，未清理该拒绝记录或外部文件。

## 显示语义与限制

- 绿色当前帧为估计位姿变换后的降采样 snapshot 点云；没有合法位姿时不捏造 map→lidar 变换。当前帧显示衰减 0.5 s。
- 点云地图是有限容量的显示累积，0.4 m 体素、最多 150,000 点；淘汰最旧体素。不修正历史点、不作为复用定位输入，不等于稠密地图保存/重定位能力。
- 两个运行间保存并复用的是原系统的相关 UWB A/L 地图；橙色标记取当前优化图中的基站变量，未初始化基站不显示猜测位置。保留 provisional 及高度不可靠限制。
- 测距文字为原始接收值和 status，包含 TA/AA；连线端点是当前基站估计和测量时刻附近（最大 60 ms）的标签估计，使用已知标签外参。缺少端点时只显示文字，接收不等于优化器接受。两秒无新测量即移除显示。
- 每 run 的 RViz 配置快照、显示脚本哈希、进程命令及日志随原运行器保存；run owner 统一管理 ROS domain、时间、看门狗和退出。退出 RViz 会使本次运行记录为失败；不会留下无期限运行的后台任务。
- 不声称 GPU GUI 渲染、rolling/去畸变、真实 Mid-360、完整无人机或部署认证通过。
