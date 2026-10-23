# 五组同输入对比工具

本目录是独立的对比工作区工具，不加入 Tunnel 的三个正式 ROS 包，不加载 IAP 私有库。源码和编译产物写入本项目 `log/baseline_*`，不改动外部工作区。

## 冻结实验约定（完整运行前声明）

- 五组：FAST-LIO2、LIO-SAM、本方法无图 UWB 联合自标定、本方法有图 UWB 复用、本方法 LiDAR+IMU（UWB OFF）。外部算法不使用 UWB/GPS/真值。LIO-SAM 关闭回环，本轮比较在线定位，而非离线回环后全局轨迹。
- 唯一评测原始输入：`log/20261022T161447Z_132`，seed 1，360 s，snapshot，40 rings × 512 水平射线，10 Hz LiDAR / 200 Hz IMU，40 m 雷达量程。物理场景、噪声与轨迹不修改。
- 已有图来自独立 seed 0 首轮 `log/20261022T160635Z_994/export/map/calibrated_map.json`，不得用 seed 1 评测数据重新建立该先验。
- 本方法有图复用 `20261022T162059Z_880`，OFF 复用 `20261022T161447Z_132`。新增无图组及外部两组；不进行多种子、不按精度挑最佳重复。
- 主要 APE 使用平移误差范数，以共同有效时刻、单次 SE(3) 刚体对齐计算，无尺度拟合。预声明主评测窗口 ROS stamp [61, 360.9] s，另报全输出时段及固定首姿态对齐误差，避免全程对齐掩盖漂移。
- 首先匹配同时间的真实位置；方法间采用共同有效样本，同时报告各自覆盖率、开始/结束时间、最大输出间隔。缺失轨迹不计为零误差，不跨长时间断档补轨迹；若无图组初始化失败，明确 N/A，不把失败当成整个算法不可行。
- 三种本方法的数学实现沿用已有冻结 GPU 路线；外部两个基线为其原生 CPU 实现。不同硬件后端不用于宣称公平的速度排名。

## 传感器适配，不提供真值先验

`run_external.py` 从同一原始 MCAP 只发布 `/benchmark/points`、`/benchmark/imu`、`/clock`，不发布真值、UWB 或 GPS。点坐标、强度、IMU 原始加速度/角速度和测量时间不改变。ring 依据已知传感器垂直射线角度 [-25°,25°] / 40 线恢复，并要求角度残差小于 0.05°。这属于传感器信息，不是隧道边界。点时间保持零。

LIO-SAM 官方接口要求姿态；本仿真无姿态传感器，故明确标记为 **六轴 IMU 适配版**：使用前 1.5 s 已声明静止段的平均比力确定初始 roll/pitch，yaw=0 仅为局部坐标规范；随后对去除该静止段平均偏置后的角速度积分，提供派生姿态。原始 gyro/acc 本身不扣偏置，两算法接收完全相同的适配消息。`orientation_covariance[0]=-1` 仍表明没有独立姿态观测。LIO-SAM `useImuHeadingInitialization=false`、`imuRPYWeight=0`，不把派生姿态当独立绝对航向/持续姿态观测。不得把本实验冒充有九轴姿态观测的原版 LIO-SAM 结果。

FAST-LIO snapshot 补丁禁止全零 time 触发其默认的方位角时间重建；等时点云跳过去畸变，IMU 传播保留。其余补丁仅为 C++17、Eigen3/GTSAM 智能指针和 LIO-SAM TF 消息 header 的系统兼容修复。没有按真值改估计值或抑制算法失败。

输出统一转换为 IMU/body 原点：FAST-LIO 输出已经是 body；LIO-SAM mapping odometry 是 LiDAR 原点，通过已知 T_BL 转回 body。传感器外参是原场景 T_BL=(yaw 0.05 rad, translation [0.2,0,0.1] m)。

## 复现

```bash
bash tools/baselines/build.sh
bash tools/baselines/run.sh fastlio
bash tools/baselines/run.sh liosam
LIUO_RUN_ROOT="$PWD/log" ./tools/run.sh \
  --replay log/20261022T161447Z_132 \
  --config packages/liuo_tunnel/config/phase3_self_calibrate.json \
  --installation packages/liuo_tunnel/config/installation/straight_simulated.json \
  --formal --wall-timeout 480
```

运行器总墙钟上限为输入长度+120 s，45 s 无输出即失败，退出只向自己创建的进程组发送信号，无无限重试。配置、原始文件哈希、适配后消息哈希、二进制哈希、源码身份、轨迹及失败原因与同一 run 保存。源码下载每份最多 300 s，无无限重试。

本轮原始运行的 CDR 点云哈希不一致；用同一个消息连续序列化已复现对齐填充字节变化，反序列化后的所有字段与点数据相等。保留原字节哈希，使用 `audit_canonical_input.py` 从两次各自的干净提交重建转换器，对相同原始文件分别计算规范化消息哈希并核对发布总数。审计 sidecar 是事后发布侧确定性重建，不是接收侧录包，不能证明算法消费了所有消息。后续运行器同时直接记录规范化哈希。复核命令：

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/baselines/audit_canonical_input.py \
  log/20261023T021753Z_515 log/20261023T022435Z_656
```

## 来源与许可证

- [FAST-LIO ROS2](https://github.com/hku-mars/FAST_LIO/tree/a4743b095409588842a5b30ddfa27e29d2f99164)：实际 LICENSE 为 GPL v2（package.xml 的 BSD 标注不能覆盖 LICENSE），保留原源码许可证。
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree/tree/e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4)：保留源码 LICENSE。
- [LIO-SAM ROS2](https://github.com/TixiaoShan/LIO-SAM/tree/08af3f32f01725372d4269838dc44c19c6d9e76b)：BSD-3-Clause，保留源码 LICENSE。
- `livox_ros_driver2/` 仅复制官方 [CustomMsg / CustomPoint 定义](https://github.com/Livox-SDK/livox_ros_driver2/tree/master/msg)及 MIT 许可证，为未使用的 Livox 输入分支提供链接类型；没有复制、启动或冒充硬件驱动。
- `sources.json` 记录从 Git archive PAX comment 核对的提交与实际下载包哈希；`patches/` 保存全部源码修改。源码目录被 gitignore，按固定提交可重建，不提交视频和大型示例资源。
