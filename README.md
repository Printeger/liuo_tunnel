# LIUO Tunnel

Independent ROS 2 Jazzy phase-one GPU LiDAR–IMU baseline. UWB is published and recorded, never fused. Development/acceptance is in progress; see `docs/development_handbook.md` for the scope.

```bash
cd /home/dev/code/ws_tunnel/src/liuo_tunnel
./tools/build.sh
./tools/test.sh
./tools/run.sh --formal --duration 45 --seed 0
./tools/run.sh --duration 20 --reset-at 10 --seed 0
./tools/run.sh --duration 20 --inject-failure odometry
python3 tools/evaluate.py log/<run_id>
```

Build/run wrappers discard inherited overlays, source `/opt/ros/jazzy`, and then load only this project's installation. Build scans only `packages/`. Dependencies: ROS 2 Jazzy, Eigen, PCL common, GTSAM, gtsam_points, spdlog, Boost filesystem, OpenMP. The required CUDA-enabled gtsam_points 1.2.1 uses public GPU binary matching factors and the extended incremental smoother. A working NVIDIA device/driver is required for the default configuration and GPU tests. Formal runs require GPU and requested rate 1.0; failures never switch to CPU. An explicit alternate configuration with odometry.backend=cpu is diagnostic only.

All commands write artifacts under the project's ignored `log/`; `LIUO_RUN_ROOT` accepts an explicit absolute override. Each live run has one owner, atomic `metadata/run_manifest.json`, copied configuration, source and binary identity, child console/spdlog logs, CSV observation/trajectory/graph/information/timing exports, and offline `export/analysis/report.json`. Failures remain in their original runs. The launcher rejects dirty source before live evidence. SIGINT stops only its own children. Tests use automatically cleaned temporary directories. Build/test tooling outputs use `log/build` and `log/test`, separate from simulation run IDs.

Frames: W is simulator world; odom starts at IMU initialization with gravity alignment and yaw gauge zero. T_BL maps LiDAR into IMU/body. IMU has unavailable orientation and measures specific force. Output pose covariance converts GTSAM right perturbations into parent-frame ROS ordering. UWB antenna translation is T_BU; no true anchor coordinates enter estimator configuration. Simulator truth is used by offline evaluation only; the observer audits estimator subscriptions.

`scan_model=snapshot` is explicit, with all point `time=0`. Rolling scan generation and IMU deskew are not validated and unsupported modes fail. The first-hit renderer returns world intersections; the simulator converts them into the actual LiDAR frame before adding range noise. The 2.4 km straight world is loaded in bounded local blocks; UWB LOS uses the entire convex corridor. Nonperiodic equipment exists only in the first 50 m; the rest contains repeated rails/sleepers and planar walls. Motion has continuous acceleration, 5 s initial standstill and intermediate stops at 600/1200/1800 m; a full run needs about 540 simulation seconds.

Online VGICP factors connect the current frame to the last three variable poses and active-window keyframes (2 m / 0.5 rad selection, at most 15). X/V/B timestamps use a 10 s lag. No fixed historical target or additional IMU-predicted-pose prior is used. The LiDAR information spectrum is diagnostic. Local covariance plane normalization requires eigenvalue ratios λ1/λ2 ≥ 0.1 and λ0/λ1 ≤ 0.1; otherwise covariance remains isotropic, avoiding invented normals on sparse scan rings/corners. This is an actual covariance change tested through the VGICP Hessian. There is no spectrum-based posterior pose weighting. UWB CSV reason `RECORD_ONLY` explicitly means no graph keys, prediction or innovation exist yet. Phase-two interfaces are raw UwbRange, fixed extrinsics, and the single estimator-owned graph update seam; no UWB factor, anchor state or map loader is implemented.

Current evidence supports only the configurations actually run, and does not certify rolling/deskew, hardware operation, UWB fusion, calibration, map reuse or planning.
