# 第一阶段验收记录（2026-10-21）

结论：按用户明确允许 snapshot、并要求正式 GPU 路线的阶段一范围，所有必需退出项 PASS。下列未验证能力和实时限制不被升级为已完成能力。

范围按用户本次要求冻结：第一阶段允许明确标记的 snapshot，rolling 留待后续；正式运行及主要开发使用 GPU。Handbook 的 rolling 最终验收要求仍有效，本文不把 snapshot 等同于完整扫描模型验收。只交付独立迁移和 LiDAR–IMU 基线，UWB 仅发布/记录。

## 来源与复现

IAP 源分支 `dev/iap_refactor`，核查提交 `fbccae16802402d7300bb817d2627f86e6a495df`；开始和结束检查均无改动。目标在自身目录初始化独立 Git 仓库，未建立额外分支/worktree。源码来源逐文件见 `../THIRD_PARTY_NOTICES.md`，许可证保留。

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
./tools/build.sh
./tools/test.sh
./tools/run.sh --formal --duration 45 --seed 0
./tools/run.sh --formal --duration 20 --reset-at 10 --seed 0
./tools/run.sh --formal --duration 20 --inject-failure odometry  # 预期非零，保留 failed run
python3 tools/evaluate.py log/<run_id>
```

构建/运行 wrapper 使用 env -i，仅 source /opt/ros/jazzy 和本项目 install；colcon 只扫描本项目 packages。三个包为 liuo_tunnel_msgs、liuo_tunnel、liuo_tunnel_sim。规范化运行入口为 tools/run.sh → run.py：分配共享 run、启动已安装程序、收集进程/动态库证据、监控失败并评估。无 IAP overlay、配置、消息包、私有库运行依赖。

依赖：ROS 2 Jazzy / rclcpp 28.1.16；GTSAM 4.3a0（CMake 4.3.0）；gtsam_points 1.2.1，安装头文件 build identity 85d0f4c，CUDA 12.5.82，OpenMP；Eigen 3.4、PCL 1.14、spdlog 1.12、Boost 1.83、nlohmann JSON 3.11.3。GPU NVIDIA GeForce RTX 4070 Ti SUPER 16 GiB，driver 580.126.09。不以 /root/gtsam_points 当前源码 HEAD 代替已安装库身份。各 run 的 input_identity.json 保存实际二进制、依赖库 SHA256 和 CMakeCache；runtime_identity.json 保存实际进程映射。

## 验收矩阵

| 项目 | 结果 | 证据 |
|---|---|---|
| 干净 underlay，仅三个 Tunnel 包独立构建 | PASS | log/gpu-build-final.txt；运行库路径/哈希审计 |
| CPU 原始多帧因子参考路径 | PASS（参考范围） | test_vgicp；历史 15 s run 20261021T080623Z_776，入口 RMSE 0.353 m；CPU 1 倍长时运行失败见下 |
| GPU 原始多帧 VGICP + IMU X/V/B 滑窗 | PASS | GPU 双位姿因子真实 CUDA 测试和正式运行；无预测位姿软先验、无配准结果位姿先验、无 fallback |
| GPU 驱动初始化/设备数 | PASS | metadata/gpu_driver.json：cuInit=0、cuDeviceGetCount=0、device_count=1；nvidia-smi 快照 |
| 真值隔离、UWB 不入图 | PASS | 运行订阅审计；graph.csv 中 A=0；uwb.csv 全部 RECORD_ONLY |
| IMU 比力/噪声密度、外参/杆臂、时间和消息字段 | PASS | test_imu、test_physics、运行 observation_audit.json |
| 799.9 / 800.0 / 800.1 m 真实三维边界 | PASS | ThreeDimensionalBoundaryAndUnclippedTail；800 m 加噪后大于 800 的样本保留 |
| LOS 高斯噪声与独立随机流 | PASS | 两个独立流各 N=20000，实际无阻挡 anchor(2)–anchor(4) 链路；见统计值 |
| 共享 run、manifest、源版本和输入身份 | PASS | RunLogManager 测试、Python 并发分配/失败优先级测试、实际子进程相同 run_directory |
| 时间重置和 session | PASS | 20261021T082926Z_422，两个 producer/估计器 session；重置后重新初始化 |
| 失败退出、保留原因 | PASS | 20261021T083033Z_044：odometry=-9，owner 非零退出，lifecycle=failed，input_failure.json 保留；非精度发散注入 |
| 入口和退化段的真实 LIO 观测 | PASS | 正式 45 s 运行覆盖真实 x>90 m；报告自然漂移，未要求/制造 LIO 崩溃 |
| snapshot 原始点云 | PASS | 512×40 波束、10 Hz，time=0，明确 scan_model=snapshot |
| rolling / 去畸变 | 未验证/未实现 | 用户允许阶段一 snapshot；未声称通过 rolling |
| 全系统 1 倍墙钟实时 | 未通过 | GPU 求解时延和仿真渲染墙钟倍率分开报告，不能把请求 rate=1 当成实测 1 倍 |
| UWB 融合、自标定、地图复用、规划、真实硬件 | 未验证/未实现 | 不属于阶段一，未进入第二/第三阶段 |

测试汇总：25 项 colcon 测试，0 errors / 0 failures / 0 skipped；3 项 Python 合同测试通过。日志 log/gpu-tests-final.txt。高斯样本固定 seed=0：流 100 mean=0.000777862 m、sd=0.100734 m；流 200 mean=-0.00134481 m、sd=0.0994148 m。均值上界 0.00212132 m，标准差范围 [0.095,0.105] m。未裁剪或重选种子。

## 实际运行与性能

最终正式 run：`20261021T083058Z_660`，源码 cef77d1，lifecycle=completed、acceptance=pass。实际目录 `/home/dev/code/ws_tunnel/src/liuo_tunnel/log/20261021T083058Z_660`。入口 181 帧 RMSE **0.182666 m**，过渡段 79 帧 RMSE 0.721520 m，退化段 170 帧 RMSE **8.710140 m**，最大轴向误差 **17.425451 m**。姿态 P95 1.019437°；处理 P95 **48.6088 ms**；模拟时间输出延迟 P95 0.055 s；最多 101 个车辆状态。原始观测 9000 IMU / 450 cloud / 5525 UWB，审计无违规。45 s 模拟耗时 74.585117 s，实测倍率 **0.603338**，故不宣称全系统 1 倍墙钟实时。单帧 GPU 处理可以跟上本次输入，不等价于所有工况实时保证。

该运行与本任务其他验证 run 不重叠；系统桌面/其他用户 GPU 进程保持原状。首次和补跑的同种子结果有小幅浮点/增量求解差异，不声称 CUDA 归约位级确定性。固定种子保证原始传感器随机流可复现。

初次 GPU 运行 20261021T082822Z_591：入口 RMSE 0.182714 m、退化段 RMSE 8.671781 m、最大轴向误差 17.334161 m；处理 P95 48.6039 ms，仿真倍率 0.607328。后半段与 session 测试有部分重叠，不能作为独占性能测量。session 测试 20261021T082926Z_422：4000 IMU / 200 cloud / 2686 UWB，两个 session，入口 RMSE 0.121804 m，PASS。

所有 run 位于本项目 log/<run_id>，正式 run 为 protected。关键文件：metadata/run_manifest.json、metadata/config/、metadata/manifests/{input_identity,runtime_identity,observation_audit}.json、metadata/processes/odometry.json、runtime/liuo_odometry.log、export/odometry/{trajectory,graph,icp,uwb}.csv、profiling/odometry.csv、export/analysis/{report.json,report.md,metrics.csv}。truth 只用于独立评估，固定初始化 z=-1.2 m gauge，不做轨迹拟合。LiDAR 谱为混合单位位姿块诊断，不是严格协方差覆盖率保证。

## 失败记录及修复依据

- 20261021T074448Z_313：ROS 参数类型失败，修正 reset_at 的浮点默认值。
- 20261021T074513Z_572：历史报告误标 PASS，只检查进程/数据合同，实际入口 RMSE 22.779 m。该 PASS **无效**，由后续入口 RMSE≤1 m 的基本跟踪检查明确取代，原始证据不覆写。
- 20261021T075045Z_805、075314Z_519、075631Z_465、075943Z_605、080318Z_561：入口跟踪 FAIL。诊断涉及原始点云、IMU、VGICP Hessian，依次修复浮点体素边界造成的隧道墙孔洞、无平面支持的局部协方差，以及迁移遗漏的活动窗口关键帧连接。单独收紧初始 bias 或增加邻域并未解决，保留各次失败。
- 20261021T081707Z_456：CPU 45 s 运行触发 cloud_queue_limit，未完成退化段。20261021T081928Z_785：0.5 m 降采样尝试仍超队列且损伤跟踪，已回退到 0.25 m。没有增加队列掩盖积压或把失败降为通过。
- 当前 GPU 路径复用公共 gtsam_points GPU 原始因子与批量线性化 hooks；不移植 GNSS/ARAIM 扩展。自然轴向漂移保留，不用真值、轨道位姿限制或人为先验修饰输出。

## 后续接口与限制

第二阶段可接入现有 UwbRange producer/session/sequence 与每链路有效测量时间、T_BU 杆臂、同一估计器的 X/V/B 图更新入口；尚无 UWB 图键、插值因子、基站状态、地图装载或标定器。非零 point.time/rolling 模式明确拒绝，不能把未实现的去畸变当成有效输入。当前场景为直隧道，曲线运动比力仅有解析单元测试，未声称曲线隧道渲染已验证。完整 2350 m 行程与所有中途停车是模型能力，本阶段实际验收运行仅覆盖入口和早期退化段。

构建和运行的外部安装依赖需按记录部署；本项目不随仓打包 ROS/CUDA/GTSAM。raw bag 默认关闭，原始 IMU/UWB 已记录、点云仅在 --diagnostic-capture 时额外保存选定 CDR 快照；未声称已有完整 bag 回放验收。

主要提交：d740a03 独立工程；e3ad44f 实际墙体回归修复；c1677c0 协方差平面支持修复；e26324e 恢复多帧关键帧连接；8f7a24d 传感器/运行合同；66877cf GPU 主路径；bb8f3b1 驱动预检；cef77d1 真实 LOS 统计测试。完整记录使用 git log。最终文档提交记录见仓库 HEAD。
