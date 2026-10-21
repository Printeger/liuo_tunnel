// Adapted from IAP apps/iap_rosnode.cpp and odometry_estimation_{imu,cpu}.cpp @ fbccae168.
// Retains X/V/B fixed-lag IMU + direct multi-frame VGICP; removes GNSS, plugins, ARAIM,
// scan-to-map pose priors and the redundant per-frame IMU prediction soft prior.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <liuo_tunnel_msgs/msg/uwb_range.hpp>
#include <liuo_tunnel/common/imu_integration.hpp>
#include <liuo_tunnel/common/cloud_covariance_estimation.hpp>
#include <liuo_tunnel/common/sensors.hpp>
#include <liuo_tunnel/common/state_covariance.hpp>
#include <liuo_tunnel/common/gauge_consistent_vgicp_gpu.hpp>
#include <liuo_tunnel/uwb/known_map.hpp>
#include <liuo_tunnel/uwb/measurement_queue.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <liuo_tunnel/util/run_log_manager.hpp>
#include <liuo_tunnel/util/logging.hpp>
#include <liuo_tunnel/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <cuda_runtime_api.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <Eigen/Eigenvalues>
#include <deque>
#include <set>
#include <fstream>
#include <iomanip>
using gtsam::symbol_shorthand::A;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
namespace liuo_tunnel {
class TunnelNode : public rclcpp::Node {
public:
  TunnelNode() : Node("liuo_tunnel_node") {
    cfg = load_config(declare_parameter<std::string>("config_file", ""));
    backend = cfg.at("odometry").at("backend");
    if (backend != "gpu" && backend != "cpu") throw std::runtime_error("unknown backend");
    if (backend == "gpu") {
      int devices = 0;
      auto result = cudaGetDeviceCount(&devices);
      if (result != cudaSuccess || devices < 1) throw std::runtime_error(std::string("GPU required: ") + cudaGetErrorString(result));
      cudaDeviceProp props{};
      if (cudaGetDeviceProperties(&props, 0) != cudaSuccess) throw std::runtime_error("GPU device properties unavailable");
      gpu_name = props.name;
      streams = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
    }
    T_BL = lidar_extrinsic(cfg);
    fuse = cfg.at("uwb").at("mode") == "fuse";
    if (fuse) {
      tag = vector3(cfg.at("sensors").at("T_BU_translation"));
      map = KnownMap::load(declare_parameter<std::string>("map_file", ""), tag);
    }
    logger = create_module_logger("odometry");
    auto& logs = RunLogManager::instance();
    logs.write_run_info({{"backend", backend + "_RAW_MULTIFRAME_VGICP"}, {"uwb", fuse ? "FUSED_RAW" : "RECORD_ONLY"}, {"gpu_device", gpu_name}});
    trajectory.open(logs.export_path("odometry/trajectory.csv"));
    trajectory << std::setprecision(16) << "session,stamp,publish_stamp,x,y,z,qx,qy,qz,qw,vx,vy,vz,cov_valid,state,map_id,publish_wall_ns\n";
    covariance_log.open(logs.export_path("odometry/covariance.csv"));
    covariance_log << std::setprecision(16) << "session,stamp";
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++) covariance_log << ",xv_" << i << "_" << j;
    covariance_log << '\n';
    graph.open(logs.export_path("odometry/graph.csv"));
    graph << "session,stamp,X,V,B,A,imu_factors,lidar_factors,total_factors,marginalized,optimization_ms,reset,fallback,uwb_ta,uwb_aa,map_priors,uwb_queue\n";
    icp.open(logs.export_path("odometry/icp.csv"));
    icp << "session,stamp,points,inlier_fraction,cost,eig0,eig1,eig2,eig3,eig4,eig5,weak_rx,weak_ry,weak_rz,weak_tx,weak_ty,weak_tz,weighting,nonplanar_covariances\n";
    uwb.open(logs.export_path("odometry/uwb.csv"));
    uwb << std::setprecision(16)
        << "producer,session,sequence,measurement_stamp,receive_stamp,a_type,a,b_type,b,range,variance,reason,key_left,key_right,key_anchor,residual_class,residual,whitened,"
           "weight\n";
    timing.open(logs.profiling_path("odometry.csv"));
    timing << "session,stamp,preprocess_ms,optimize_ms,total_ms,latency_s,imu_queue,cloud_queue,pred_x,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z\n";
    reset();
    pub = create_publisher<nav_msgs::msg::Odometry>("/liuo/odom", 100);
    diagnostics = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/liuo/diagnostics", 10);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>("/sensors/imu", rclcpp::SensorDataQoS().keep_last(2048), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { on_imu(*m); });
    cloud_sub =
      create_subscription<sensor_msgs::msg::PointCloud2>("/sensors/lidar/points", rclcpp::SensorDataQoS().keep_last(32), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        if (clouds.size() >= cfg.at("odometry").at("cloud_queue_limit").get<size_t>()) throw std::runtime_error("cloud_queue_limit exceeded");
        clouds.push_back(m);
      });
    uwb_sub = create_subscription<liuo_tunnel_msgs::msg::UwbRange>(
      "/sensors/uwb/range",
      rclcpp::SensorDataQoS().keep_last(2048),
      [this](liuo_tunnel_msgs::msg::UwbRange::ConstSharedPtr m) { on_uwb(*m); });
    estimator_timer = create_wall_timer(std::chrono::milliseconds(1), [this] { drain(); });
    logger->info("{} raw multi-frame VGICP ready; device={}; UWB={}; truth is not subscribed", backend, gpu_name, fuse ? "FUSED_RAW" : "RECORD_ONLY");
  }

private:
  void reset() {
    gtsam::ISAM2Params p;
    p.relinearizeSkip = 1;
    p.relinearizeThreshold = .05;
    p.findUnusedFactorSlots = true;
    if (cfg.at("odometry").value("use_dogleg", false)) p.setOptimizationParams(gtsam::ISAM2DoglegParams());
    p.setFactorization(cfg.at("odometry").value("factorization", std::string("CHOLESKY")));
    smoother = std::make_unique<liuo_tunnel::solver::IncrementalFixedLagSmootherExt>(cfg.at("odometry").at("lag_s"), p);
    IMUIntegrationParams ip;
    double rate = cfg.at("sensors").at("imu_rate_hz");
    ip.acc_noise = cfg.at("sensors").at("sigma_acc_sample").get<double>() / std::sqrt(rate);
    ip.gyro_noise = cfg.at("sensors").at("sigma_gyro_sample").get<double>() / std::sqrt(rate);
    imu = std::make_unique<IMUIntegration>(ip);
    for (const auto& old : range_queue.reset()) log_range(old, "CLOCK_RESET");
    bootstrap_factors.resize(0);
    bootstrap_values.clear();
    bootstrap_times.clear();
    bootstrapped = !fuse;
    clouds.clear();
    frames.clear();
    count = 0;
    initialized = false;
    first_imu = -1;
    latest_imu = -1;
    last_cloud = -1;
    sum_a.setZero();
    sum_w.setZero();
    initial_samples = 0;
    session++;
    last_uwb = -1;
  }
  void status(const std::string& state, const std::string& reason, double t) {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "liuo/odometry";
    d.hardware_id = backend;
    d.level = state == "TRACKING" ? 0 : 1;
    d.message = state + ":" + reason;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "measurement_stamp";
    kv.value = std::to_string(t);
    d.values.push_back(kv);
    a.status.push_back(d);
    diagnostics->publish(a);
    if (state != last_status) {
      logger->info("state={} reason={} stamp={} session={}", state, reason, t, session);
      last_status = state;
    }
  }
  void on_imu(const sensor_msgs::msg::Imu& m) {
    double t = rclcpp::Time(m.header.stamp).seconds();
    if (latest_imu >= 0 && t < latest_imu - 1e-6) {
      logger->warn("clock reset {} -> {}; start new estimator session", latest_imu, t);
      reset();
    }
    if (t == latest_imu) return;
    if (m.header.frame_id != "imu" || m.orientation_covariance[0] != -1) throw std::runtime_error("IMU frame/orientation contract violated");
    Eigen::Vector3d a(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z), w(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    if (!a.allFinite() || !w.allFinite() || t <= 0) throw std::runtime_error("invalid IMU measurement");
    if (latest_imu > 0 && t - latest_imu > .02) throw std::runtime_error("IMU gap exceeds 20ms; cannot fabricate preintegration");
    if (first_imu < 0) first_imu = t;
    latest_imu = t;
    imu->insert_imu(t, a, w);
    if (!initialized) {
      sum_a += a;
      sum_w += w;
      initial_samples++;
    }
    if (imu->imu_data_in_queue().size() > cfg.at("odometry").at("imu_queue_limit").get<size_t>()) throw std::runtime_error("imu_queue_limit exceeded");
  }
  void log_range(
    const RangeObservation& o,
    const std::string& reason,
    const RangeBracket& bracket = {},
    const std::string& anchor_key = "",
    double residual = NAN,
    const std::string& category = "NONE") {
    uwb << o.producer << ',' << o.session << ',' << o.sequence << ',' << o.stamp << ',' << o.received << ',' << int(o.a_type) << ',' << o.a << ',' << int(o.b_type) << ',' << o.b
        << ',' << o.range << ',' << o.variance << ',' << reason << ',';
    if (bracket.reason == "SYNC" || bracket.reason == "INTERPOLATED") uwb << gtsam::DefaultKeyFormatter(bracket.left);
    uwb << ',';
    if (bracket.right) uwb << gtsam::DefaultKeyFormatter(*bracket.right);
    uwb << ',' << anchor_key << ',' << category << ',' << residual << ',' << residual / std::sqrt(o.variance) << ",1\n";
  }
  void on_uwb(const liuo_tunnel_msgs::msg::UwbRange& m) {
    RangeObservation o{
      m.producer_id,
      m.session_id,
      m.sequence,
      rclcpp::Time(m.header.stamp).seconds(),
      now().seconds(),
      m.range_m,
      m.variance_m2,
      m.node_a_type,
      m.node_b_type,
      m.node_a_id,
      m.node_b_id};
    if (m.status != 0 || !m.header.frame_id.empty()) {
      log_range(o, "INVALID_STATUS_OR_FRAME");
      return;
    }
    if (!fuse) {
      log_range(o, "RECORD_ONLY");
      return;
    }
    if (o.a_type == 1 && o.b_type == 0) {
      std::swap(o.a_type, o.b_type);
      std::swap(o.a, o.b);
    }
    if (o.a_type == 0 && o.a != 0) {
      log_range(o, "UNKNOWN_TAG");
      return;
    }
    try {
      map.index(o.b);
      if (o.a_type == 1) map.index(o.a);
    } catch (const std::exception&) {
      log_range(o, "UNKNOWN_ANCHOR");
      return;
    }
    auto reason = range_queue.push(o);
    if (reason != "QUEUED") log_range(o, reason);
  }
  void append_ranges(double t, gtsam::NonlinearFactorGraph& factors, const gtsam::Values& predicted) {
    new_ta = 0;
    new_aa = 0;
    std::map<double, gtsam::Key> poses;
    for (const auto& frame : frames) poses[frame.t] = X(frame.id);
    poses[t] = X(count);
    for (const auto& o : range_queue.take_until(t)) {
      auto bracket = range_bracket(poses, o.stamp);
      if (bracket.reason != "SYNC" && bracket.reason != "INTERPOLATED") {
        log_range(o, bracket.reason);
        continue;
      }
      const size_t j = map.index(o.b);
      if (o.a_type == 1) {
        const size_t i = map.index(o.a);
        if (!map.surveyed()) {
          log_range(o, "ORACLE_AA_CHECK", {}, "", o.range - (map.means[i] - map.means[j]).norm(), "CONSISTENCY_ONLY");
          continue;
        }
        auto f = gtsam::make_shared<AnchorAnchorRangeFactor>(A(o.a), A(o.b), o.range, std::sqrt(o.variance));
        factors.add(f);
        new_aa++;
        log_range(o, "ACCEPT_AA", {}, gtsam::DefaultKeyFormatter(A(o.a)) + ";" + gtsam::DefaultKeyFormatter(A(o.b)), f->unwhitenedError(predicted)[0], "PRE_OPT_INNOVATION");
      } else {
        auto anchor = map.surveyed() ? std::optional<gtsam::Key>(A(o.b)) : std::nullopt;
        auto f = gtsam::make_shared<TagAnchorRangeFactor>(bracket.left, bracket.right, anchor, map.means[j], tag, bracket.beta, o.range, std::sqrt(o.variance));
        double residual = f->unwhitenedError(predicted)[0];
        factors.add(f);
        new_ta++;
        last_uwb = std::max(last_uwb, o.stamp);
        log_range(o, "ACCEPT_TA", bracket, anchor ? gtsam::DefaultKeyFormatter(*anchor) : "constant:" + std::to_string(o.b), residual, "PRE_OPT_INNOVATION");
      }
    }
  }
  void drain() {
    while (!clouds.empty()) {
      double t = rclcpp::Time(clouds.front()->header.stamp).seconds();
      if (latest_imu < t + 1e-8) return;
      auto cloud = clouds.front();
      clouds.pop_front();
      if (t <= last_cloud) {
        status("DEGRADED", "OUT_OF_ORDER_CLOUD", t);
        continue;
      }
      if (t < first_imu) continue;
      if (!initialized && t - first_imu < cfg.at("odometry").at("initialization_s").get<double>()) {
        status("INITIALIZING", "IMU_STATIC_ALIGNMENT", t);
        continue;
      }
      process(*cloud, t);
    }
  }
  struct Frame {
    size_t id;
    double t;
    gtsam_points::GaussianVoxelMap::Ptr voxels;
    bool keyframe;
  };
  void process(const sensor_msgs::msg::PointCloud2& m, double t) {
    auto start = std::chrono::steady_clock::now();
    if (m.header.frame_id != "lidar" || m.is_bigendian) throw std::runtime_error("pointcloud frame/endian contract");
    std::vector<Eigen::Vector4d> points;
    std::set<std::tuple<int, int, int>> occupied;
    double leaf = cfg.at("odometry").at("downsample_m");
    sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z"), dt(m, "time"), intensity(m, "intensity");
    for (; x != x.end(); ++x, ++y, ++z, ++dt, ++intensity) {
      if (*dt != 0) throw std::runtime_error("nonzero per-point time with snapshot model; rolling not implemented");
      Eigen::Vector3d q(*x, *y, *z);
      if (!q.allFinite()) continue;
      q = T_BL * q;
      auto key = std::make_tuple(int(std::floor(q.x() / leaf)), int(std::floor(q.y() / leaf)), int(std::floor(q.z() / leaf)));
      if (occupied.insert(key).second) points.emplace_back(q.x(), q.y(), q.z(), 1);
    }
    if (points.size() < 100) throw std::runtime_error("insufficient finite cloud points");
    gtsam_points::KdTree tree(points.data(), points.size());
    const size_t k = cfg.at("odometry").at("covariance_neighbors");
    std::vector<int> neighbors(points.size() * k);
    for (size_t i = 0; i < points.size(); i++) {
      std::vector<size_t> ids(k);
      std::vector<double> distances(k);
      if (tree.knn_search(points[i].data(), k, ids.data(), distances.data()) != k) throw std::runtime_error("covariance neighborhood missing");
      for (size_t j = 0; j < k; j++) neighbors[i * k + j] = ids[j];
    }
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
    CloudCovarianceEstimation covariance(2, cfg.at("odometry").at("validate_covariance_planarity"));
    auto covariances = covariance.estimate(points, neighbors);
    size_t nonplanar = 0;
    for (const auto& c : covariances)
      if (c.trace() > 2.9) nonplanar++;
    cloud->add_covs(covariances);
    gtsam_points::PointCloud::Ptr match_cloud = cloud;
    gtsam_points::GaussianVoxelMap::Ptr voxel;
    double resolution = cfg.at("odometry").at("voxel_resolution");
    if (backend == "gpu") {
      match_cloud = gtsam_points::PointCloudGPU::clone(*cloud);
      voxel = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(resolution);
    } else
      voxel = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
    voxel->insert(*match_cloud);
    auto preprocessed = std::chrono::steady_clock::now();
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph factors, lidar;
    if (!initialized) {
      Eigen::Vector3d avg = sum_a / initial_samples;
      if (std::abs(avg.norm() - 9.80665) > .3 || (sum_w / initial_samples).norm() > .1) throw std::runtime_error("initialization requires stationary IMU interval");
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(avg, Eigen::Vector3d(0, 0, 9.80665));
      auto guess = fuse ? vector3(cfg.at("uwb").at("initial_position_guess")) : gtsam::Vector3::Zero();
      state = gtsam::NavState(gtsam::Pose3(gtsam::Rot3(q), guess), gtsam::Vector3::Zero());
      bias = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), sum_w / initial_samples);
      if (fuse) {
        factors.emplace_shared<GravityAlignmentFactor>(X(0), avg, .001);
        if (map.surveyed()) {
          gtsam::KeyVector keys;
          for (size_t j = 0; j < map.ids.size(); j++) {
            keys.push_back(A(map.ids[j]));
            initial.insert(keys.back(), map.means[j]);
          }
          factors.emplace_shared<SurveyedMapPrior>(keys, map.means, map.covariance);
        }
      } else
        factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), state.pose(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), state.velocity(), gtsam::noiseModel::Isotropic::Sigma(3, .01));
      gtsam::Vector6 sig;
      sig.head<3>().setConstant(cfg.at("sensors").at("initial_acc_bias_sigma"));
      sig.tail<3>().setConstant(cfg.at("sensors").at("initial_gyro_bias_sigma"));
      factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), bias, gtsam::noiseModel::Diagonal::Sigmas(sig));
      initialized = true;
    } else {
      int integrated = 0;
      int erase = imu->integrate_imu(last_cloud, t, bias, &integrated);
      if (integrated < 1) throw std::runtime_error("no IMU samples for frame interval");
      state = imu->integrated_measurements().predict(state, bias);
      factors.emplace_shared<gtsam::ImuFactor>(X(count - 1), V(count - 1), X(count), V(count), B(count - 1), imu->integrated_measurements());
      gtsam::Vector6 sig;
      sig.head<3>().setConstant(cfg.at("sensors").at("sigma_ba_density").get<double>() * std::sqrt(t - last_cloud));
      sig.tail<3>().setConstant(cfg.at("sensors").at("sigma_bg_density").get<double>() * std::sqrt(t - last_cloud));
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(count - 1), B(count), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Diagonal::Sigmas(sig));
      imu->erase_imu_data(std::max(0, erase - 1));
      int window = cfg.at("odometry").at("connection_window");
      for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (int(count - it->id) > window && !it->keyframe) continue;
        gtsam::NonlinearFactor::shared_ptr f;
        if (backend == "gpu") {
          auto sb = streams->get_stream_buffer();
          f = gtsam::make_shared<GaugeConsistentVGICPFactorGPU>(X(it->id), X(count), it->voxels, match_cloud, sb.first, sb.second);
        } else {
          auto cpu = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(X(it->id), X(count), it->voxels, match_cloud);
          cpu->set_num_threads(cfg.at("odometry").at("threads"));
          f = cpu;
        }
        lidar.add(f);
        factors.add(f);
      }
    }
    double pred_x = state.position().x();
    initial.insert(X(count), state.pose());
    initial.insert(V(count), state.velocity());
    initial.insert(B(count), bias);
    gtsam::Values predicted = bootstrapped ? smoother->calculateEstimate() : bootstrap_values;
    predicted.insert(initial);
    if (fuse) append_ranges(t, factors, predicted);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    double fraction = 0, cost = 0;
    gtsam_points::NonlinearFactorSetGPU gpu_factors;
    if (backend == "gpu") {
      gpu_factors.add(lidar);
      gpu_factors.linearize(predicted);
      gpu_factors.error(predicted);
    }
    for (const auto& f : lidar) {
      gtsam::HessianFactor h(*f->linearize(predicted));
      H += h.hessianBlockDiagonal().at(X(count));
      cost += f->error(predicted);
      fraction += backend == "gpu" ? dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get())->inlier_fraction()
                                   : dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(f.get())->inlier_fraction();
    }
    if (!lidar.empty()) fraction /= lidar.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(H);
    auto eigen = eig.eigenvalues();
    auto weak = eig.eigenvectors().col(0).eval();
    gtsam::FixedLagSmoother::KeyTimestampMap times{{X(count), t}, {V(count), t}, {B(count), t}};
    if (fuse && map.surveyed())
      for (auto id : map.ids) times[A(id)] = t;
    if (!bootstrapped) {
      bootstrap_factors.add(factors);
      bootstrap_values.insert(initial);
      for (const auto& kv : times) bootstrap_times[kv.first] = kv.second;
      const double bootstrap_duration = cfg.at("uwb").at("bootstrap_s");
      if (t - first_imu < bootstrap_duration) {
        frames.push_back({count, t, voxel, count % 10 == 0});
        last_cloud = t;
        count++;
        status("INITIALIZING", "MAP_FRAME_JOINT_BOOTSTRAP", t);
        return;
      }
      gtsam_points::LevenbergMarquardtExtParams params;
      params.maxIterations = 30;
      auto optimized = gtsam_points::LevenbergMarquardtOptimizerExt(bootstrap_factors, bootstrap_values, params).optimize();
      // Transfer the original factors and their joint optimized values exactly
      // once. No marginal-map + reused-observation double counting.
      smoother->update(bootstrap_factors, optimized, bootstrap_times);
      bootstrap_factors.resize(0);
      bootstrap_values.clear();
      bootstrap_times.clear();
      bootstrapped = true;
    } else
      smoother->update(factors, initial, times);
    state = gtsam::NavState(smoother->calculateEstimate<gtsam::Pose3>(X(count)), smoother->calculateEstimate<gtsam::Vector3>(V(count)));
    bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(count));
    if (!state.pose().matrix().allFinite() || !state.velocity().allFinite()) throw std::runtime_error("nonfinite optimized state");
    auto xv_cov = joint_covariance(smoother->getISAM2(), X(count), V(count));
    gtsam::Matrix66 cov = xv_cov.topLeftCorner<6, 6>();
    auto velocity_H = body_velocity_jacobian(state.pose(), state.velocity());
    gtsam::Matrix33 body_velocity_cov = velocity_H * xv_cov * velocity_H.transpose();
    if (!cov.allFinite() || cov.diagonal().minCoeff() < 0) throw std::runtime_error("invalid pose covariance");
    double lag = cfg.at("odometry").at("lag_s");
    size_t retired = 0;
    while (!frames.empty() && frames.front().t < t - lag) {
      frames.pop_front();
      retired++;
    }
    bool keyframe = frames.empty();
    auto last_key = std::find_if(frames.rbegin(), frames.rend(), [](const Frame& f) { return f.keyframe; });
    if (last_key == frames.rend())
      keyframe = true;
    else {
      auto key_pose = smoother->calculateEstimate<gtsam::Pose3>(X(last_key->id));
      auto delta = key_pose.between(state.pose());
      keyframe = delta.translation().norm() > 2.0 || gtsam::Rot3::Logmap(delta.rotation()).norm() > .5;
    }
    frames.push_back({count, t, voxel, keyframe});
    size_t num_keys = std::count_if(frames.begin(), frames.end(), [](const Frame& f) { return f.keyframe; });
    for (auto& f : frames) {
      if (num_keys <= 15) break;
      if (f.keyframe) {
        f.keyframe = false;
        num_keys--;
      }
    }
    auto finish = std::chrono::steady_clock::now();
    auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    std::string mode = lidar.empty() ? "INITIALIZING" : (fraction < .5 || eigen[0] < 1e-6 || eigen[5] / std::max(eigen[0], 1e-12) > 1e6 ? "DEGRADED" : "TRACKING");
    if (fuse) {
      if (t - last_uwb > .3)
        mode = "DEGRADED";
      else
        mode = cov.bottomRightCorner<3, 3>().trace() > .25 ? "DEGRADED" : "TRACKING";
      if (cov.trace() > 100) mode = "LOST";
    }
    nav_msgs::msg::Odometry out;
    out.header.stamp = m.header.stamp;
    out.header.frame_id = fuse ? "map" : "odom";
    out.child_frame_id = "imu";
    auto p = state.position();
    auto q = state.pose().rotation().toQuaternion();
    auto v = state.velocity();
    out.pose.pose.position.x = p.x();
    out.pose.pose.position.y = p.y();
    out.pose.pose.position.z = p.z();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    // ROS pose covariance uses [translation, rotation], GTSAM right perturbation [rotation, translation]. Rotate both blocks into parent frame.
    auto ros_cov = parent_pose_covariance(state.pose(), cov);
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) out.pose.covariance[i * 6 + j] = ros_cov(i, j);
    auto body_v = state.pose().rotation().unrotate(v);
    out.twist.twist.linear.x = body_v.x();
    out.twist.twist.linear.y = body_v.y();
    out.twist.twist.linear.z = body_v.z();
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) out.twist.covariance[i * 6 + j] = body_velocity_cov(i, j);
    pub->publish(out);
    auto publish_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    status(mode, fuse ? (t - last_uwb > .3 ? "UWB_ABSENT_LIO_CONTINUES" : "JOINT_LIDAR_IMU_UWB") : "LIDAR_INFORMATION_DIAGNOSTIC_NO_DOWNWEIGHTING", t);
    trajectory << session << ',' << t << ',' << now().seconds() << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
               << ',' << v.x() << ',' << v.y() << ',' << v.z() << ",1," << mode << "," << (fuse ? map.id : "none") << "," << publish_wall_ns << "\n";
    covariance_log << session << ',' << t;
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++) covariance_log << ',' << xv_cov(i, j);
    covariance_log << '\n';
    size_t anchors = fuse && map.surveyed() ? map.ids.size() : 0;
    size_t active = (smoother->timestamps().size() - anchors) / 3;
    graph << session << ',' << t << ',' << active << ',' << active << ',' << active << ',' << anchors << ',' << (count ? 1 : 0) << ',' << lidar.size() << ','
          << smoother->getFactors().nrFactors() << ',' << retired << ',' << ms(preprocessed, finish) << ',' << (session - 1) << ",0," << new_ta << ',' << new_aa << ','
          << (anchors ? 1 : 0) << ',' << range_queue.size() << "\n";
    icp << session << ',' << t << ',' << points.size() << ',' << fraction << ',' << cost;
    for (int i = 0; i < 6; i++) icp << ',' << eigen[i];
    for (int i = 0; i < 6; i++) icp << ',' << weak[i];
    icp << ",covariance_planarity_guard," << nonplanar << '\n';
    timing << session << ',' << t << ',' << ms(start, preprocessed) << ',' << ms(preprocessed, finish) << ',' << ms(start, finish) << ',' << now().seconds() - t << ','
           << imu->imu_data_in_queue().size() << ',' << clouds.size() << ',' << pred_x;
    for (int i = 0; i < 6; i++) timing << ',' << bias.vector()[i];
    timing << '\n';
    last_cloud = t;
    count++;
  }
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> streams;
  std::string backend, gpu_name;
  bool fuse = false, bootstrapped = true;
  KnownMap map;
  gtsam::Point3 tag;
  RangeQueue range_queue;
  double last_uwb = -1;
  size_t new_ta = 0, new_aa = 0;
  gtsam::NonlinearFactorGraph bootstrap_factors;
  gtsam::Values bootstrap_values;
  gtsam::FixedLagSmoother::KeyTimestampMap bootstrap_times;
  rclcpp::TimerBase::SharedPtr estimator_timer;
  nlohmann::json cfg;
  Eigen::Isometry3d T_BL;
  std::unique_ptr<IMUIntegration> imu;
  std::unique_ptr<liuo_tunnel::solver::IncrementalFixedLagSmootherExt> smoother;
  gtsam::NavState state;
  gtsam::imuBias::ConstantBias bias;
  std::deque<Frame> frames;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;
  size_t count = 0, session = 0, initial_samples = 0;
  bool initialized = false;
  double first_imu = -1, latest_imu = -1, last_cloud = -1;
  Eigen::Vector3d sum_a, sum_w;
  std::ofstream trajectory, graph, icp, uwb, timing, covariance_log;
  std::shared_ptr<spdlog::logger> logger;
  std::string last_status;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Subscription<liuo_tunnel_msgs::msg::UwbRange>::SharedPtr uwb_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
};
}  // namespace liuo_tunnel
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  gtsam_points::LinearizationHook::register_hook([] { return std::make_shared<gtsam_points::NonlinearFactorSetGPU>(); });
  try {
    liuo_tunnel::RunLogManager::initialize("odometry");
    rclcpp::spin(std::make_shared<liuo_tunnel::TunnelNode>());
    return 0;
  } catch (const std::exception& e) {
    spdlog::error("odometry failed: {}", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
