# Migration provenance

Source: IAP `dev/iap_refactor`, commit `fbccae16802402d7300bb817d2627f86e6a495df`, clean at inspection. MIT license retained in LICENSE. Original author notices are preserved.
- `packages/liuo_tunnel/include/liuo_tunnel/util/config.hpp` ← `include/iap/util/config.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/src/util/config.cpp` ← `src/iap/util/config.cpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/include/liuo_tunnel/util/config_impl.hpp` ← `include/iap/util/config_impl.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/include/liuo_tunnel/util/logging.hpp` ← `include/iap/util/logging.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/src/util/logging.cpp` ← `src/iap/util/logging.cpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/include/liuo_tunnel/util/run_log_manager.hpp` ← `include/iap/util/run_log_manager.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/src/util/run_log_manager.cpp` ← `src/iap/util/run_log_manager.cpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/include/liuo_tunnel/common/cloud_covariance_estimation.hpp` ← `include/iap/common/cloud_covariance_estimation.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/src/common/cloud_covariance_estimation.cpp` ← `src/iap/common/cloud_covariance_estimation.cpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/include/liuo_tunnel/common/imu_integration.hpp` ← `include/iap/common/imu_integration.hpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel/src/common/imu_integration.cpp` ← `src/iap/common/imu_integration.cpp`; independent namespace, run variables and paths.
- `packages/liuo_tunnel_sim/include/liuo_tunnel_sim/first_hit_lidar_renderer.hpp` ← `src/uav_simulator/local_sensing/include/local_sensing/first_hit_lidar_renderer.hpp`; original first-hit DDA algorithm.
- `packages/liuo_tunnel_sim/src/first_hit_lidar_renderer.cpp` ← `src/uav_simulator/local_sensing/src/first_hit_lidar_renderer.cpp`; original first-hit DDA algorithm.
- `packages/liuo_tunnel/include/nlohmann/` ← IAP `thirdparty/json/include/nlohmann/`; nlohmann JSON 3.11.3, MIT, Copyright 2013–2023 Niels Lohmann. Files retain SPDX notices. MIT permission text is in LICENSE; original copyright remains in each header.
- `util/convert_to_string.hpp` ← IAP same include path; namespace only.
- `apps/liuo_tunnel_node.cpp`: reduced from `apps/iap_rosnode.cpp`, `src/iap/odometry/odometry_estimation_imu.cpp` and `odometry_estimation_cpu.cpp`. Retains initial gauge, X/V/B preintegration and direct binary VGICP; removes extensions, GNSS, ARAIM, scan-to-map registration and predicted-pose priors. Preprocessing covariance uses the migrated GLIM-derived implementation.
- `liuo_tunnel_sim/src/rail_sim.cpp`, `rail_model.hpp`: corridor surface construction adapted from `apps/demo11_corridor_map_publisher.cpp`; specific force from `quadToIapImuMsg` in `src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp`. Replaces UAV dynamics/control with continuous rail kinematics; truth orientation is never a measurement.
- `tools/run_directory.py`: `launch/_includes/run_directory.py`; generic ownership, locking, allocation, retention and atomic manifest logic retained; IAP module/clock specializations removed.
- Artifact tests and first-hit renderer tests migrated from corresponding IAP `test/` and `local_sensing/test/` paths. Tunnel physics and runtime audits are new.

Runtime third-party libraries are independently installed, not copied IAP libraries. GTSAM (BSD), gtsam_points (MIT, Kenji Koide), Eigen (MPL2), PCL (BSD), spdlog (MIT), Boost (BSL), ROS 2 (package-specific Apache/BSD licenses) retain their system distributions and licenses. Exact tested dependency/build identities are recorded in the acceptance report and run manifests.

GPU migration uses IAP odometry_estimation_gpu.cpp frame/stream/binary-factor orchestration at fbccae16802402d7300bb817d2627f86e6a495df. Matching uses public gtsam_points IntegratedVGICPFactorGPU (installed 1.2.1, build identity 85d0f4c), without IAP ARAIM/evidence extensions; IncrementalFixedLagSmootherExt enables batched CUDA linearization, with no fallback wrapper.

## Phase-two optimizer correction

`packages/liuo_tunnel/{include/liuo_tunnel,src}/optimizers/` adapts the three
`isam2_ext`, `isam2_ext_impl`, and `incremental_fixed_lag_smoother_ext` header/source
pairs from gtsam_points checkout `0a3c1b16a7e256721525f1fcb09588e033f85e63`.
This source identity differs from the installed GPU library build `85d0f4c`.
Original GTSAM author notices remain; license texts are retained in
`licenses/GTSAM-BSD.txt` (upstream GTSAM 4.2 LICENSE.BSD) and
`licenses/gtsam_points-MIT.txt` (source checkout LICENSE).
The local namespace prevents symbol interposition on the installed library.
Local corrections finish the old graph delta before inserting new variables,
keeping Dogleg's gradient/Newton vectors on the same key set, and restrict
pre-update marginalization-tree queries to existing keys when a first bootstrap
batch spans more than the lag. All expired keys still participate in ordering
and post-update marginalization.
The original GPU batching hook and Dogleg implementation remain linked from
installed gtsam_points. No fallback smoother or additional prior is introduced.

`common/gauge_consistent_vgicp_gpu.hpp` derives from the public native GPU factor.
It preserves the native correspondence, cost, source Hessian and gradient, and
reconstructs target/cross blocks using the relative-pose adjoint in double
precision to preserve common-motion gauge consistency. Tests compare these
preserved quantities and the gauge nullspace directly.
