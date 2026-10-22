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
#include <liuo_tunnel/common/estimator_status.hpp>
#include <liuo_tunnel/common/gauge_consistent_vgicp_gpu.hpp>
#include <liuo_tunnel/uwb/known_map.hpp>
#include <liuo_tunnel/uwb/installation_input.hpp>
#include <liuo_tunnel/uwb/installation_subset.hpp>
#include <liuo_tunnel/uwb/calibrated_map.hpp>
#include <liuo_tunnel/uwb/unknown_anchor_candidates.hpp>
#include <liuo_tunnel/uwb/joint_marginalization.hpp>
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
#include <liuo_tunnel/uwb/bootstrap_preconditioner.hpp>
#include <liuo_tunnel/uwb/lateral_initialization.hpp>
#include <liuo_tunnel/uwb/installation_factors.hpp>
#include <liuo_tunnel/uwb/body_wall_observation.hpp>
#include <liuo_tunnel/uwb/measurement_queue.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <liuo_tunnel/uwb/stepped_bootstrap_lm.hpp>
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
#include <unistd.h>
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
    try {
      T_BL = lidar_extrinsic(cfg);
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string("EXTRINSIC_INVALID: ") + e.what());
    }
    self_calibrate = cfg.at("uwb").at("mode") == "self_calibrate";
    reuse_map = cfg.at("uwb").at("mode") == "reuse_map";
    fuse = self_calibrate || reuse_map || cfg.at("uwb").at("mode") == "fuse";
    bounded_bootstrap_work = self_calibrate || reuse_map || cfg.at("uwb").value("bounded_bootstrap_work", false);
    if (fuse) {
      try {
        tag = vector3(cfg.at("sensors").at("T_BU_translation"));
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("EXTRINSIC_INVALID: ") + e.what());
      }
      try {
        const auto map_file = declare_parameter<std::string>("map_file", "");
        if (self_calibrate) {
          if (!map_file.empty()) throw std::invalid_argument("SELF_CALIBRATION_FORBIDS_MAP_INPUT");
          installation_input = InstallationInput::load(declare_parameter<std::string>("installation_file", ""));
          map.ids = installation_input->ids; // Identity validation only; no coordinates loaded.
          map.mode = "self_calibrate";
          map.id = "PENDING_SELF_CALIBRATION";
          map.frame = "odom";
        } else if (reuse_map) {
          calibrated_map = CalibratedMap::load(map_file, cfg.at("sensors"));
          map.ids = calibrated_map->ids;
          map.means = calibrated_map->means;
          map.mode = "calibrated_map";
          map.id = calibrated_map->document.at("map_id");
          map.frame = "map";
          const double angle = calibrated_map->reference[0] / 1000.;
          gtsam::Matrix2 conversion = gtsam::Matrix2::Identity();
          conversion(0,0) = .001;
          lateral_reference = LateralReference{{std::cos(angle),std::sin(angle)}, calibrated_map->reference[1],
            conversion * calibrated_map->covariance.bottomRightCorner<2,2>() * conversion, 0.};
        } else {
          map = KnownMap::load(map_file, tag);
          if (cfg.at("uwb").value("installation_seeds", false)) lateral_reference = LateralReference::fit(map);
        }
      } catch (const std::exception& e) {
        throw std::runtime_error(std::string("MAP_INVALID: ") + e.what());
      }
    }
    logger = create_module_logger("odometry");
    auto& logs = RunLogManager::instance();
    logs.write_run_info({{"backend", backend + "_RAW_MULTIFRAME_VGICP"}, {"uwb", fuse ? "FUSED_RAW" : "RECORD_ONLY"}, {"gpu_device", gpu_name}});
    status_log.open(logs.export_path("odometry/status.csv"));
    status_log << std::setprecision(16) << "session,stamp,state,reason,rejected_total,last_rejection,uwb_recovery_until\n";
    trajectory.open(logs.export_path("odometry/trajectory.csv"));
    trajectory << std::setprecision(16)
               << "session,stamp,publish_stamp,x,y,z,qx,qy,qz,qw,vx,vy,vz,cov_valid,state,map_id,publish_wall_ns,state_reason,wx,wy,wz,twist_interval_s,postfit_ta_stamp,postfit_"
                  "ta_count,postfit_ta_squared_sum,postfit_ta_limit,failed_recovery_stamp,uwb_optimistic_position_trace\n";
    bootstrap_log.open(logs.profiling_path("bootstrap.csv"));
    bootstrap_log << std::setprecision(16) << "session,iteration,total_error,cost_change,lambda,solve_success,elapsed_s,ta_error,translation_gradient_norm,x0,y0,z0,event\n";
    covariance_log.open(logs.export_path("odometry/covariance.csv"));
    covariance_log << std::setprecision(16) << "session,stamp";
    for (int i = 0; i < 9; i++)
      for (int j = 0; j < 9; j++) covariance_log << ",xv_" << i << "_" << j;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) covariance_log << ",twist_" << i << "_" << j;
    covariance_log << '\n';
    graph.open(logs.export_path("odometry/graph.csv"));
    graph << "session,stamp,X,V,B,A,imu_factors,lidar_factors,total_factors,marginalized,optimization_ms,reset,fallback,uwb_ta,uwb_aa,map_priors,uwb_queue,aa_summary_factors,aa_"
             "accumulated_measurements,joint_map_priors,installation_observation_factors\n";
    icp.open(logs.export_path("odometry/icp.csv"));
    icp
      << "session,stamp,points,inlier_fraction,cost,eig0,eig1,eig2,eig3,eig4,eig5,weak_rx,weak_ry,weak_rz,weak_tx,weak_ty,weak_tz,weighting,nonplanar_covariances,surface_eig0,"
         "surface_eig1,surface_eig2,translation_rank,gradient_rx,gradient_ry,gradient_rz,gradient_tx,gradient_ty,gradient_tz,gradient_world_x,gradient_world_y,gradient_world_z\n";
    uwb.open(logs.export_path("odometry/uwb.csv"));
    uwb << std::setprecision(16)
        << "producer,session,sequence,measurement_stamp,receive_stamp,a_type,a,b_type,b,range,variance,reason,key_left,key_right,key_anchor,residual_class,residual,whitened,"
           "weight,decision_wall_ns\n";
    resources.open(logs.profiling_path("resources.csv"));
    resources << "session,stamp,rss_kib,device_used_bytes,retained_cloud_frames,keyframes,imu_queue,cloud_queue,uwb_queue\n";
    timing.open(logs.profiling_path("odometry.csv"));
    timing << "session,stamp,preprocess_ms,optimize_ms,total_ms,latency_s,imu_queue,cloud_queue,pred_x,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,gpu_diagnostic_ms,smoother_ms,covariance_ms,"
              "pred_vx,pred_vy,pred_vz\n";
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
    failure_context = "SENSOR_INPUT_FAILED";
    logger->info("{} raw multi-frame VGICP ready; device={}; UWB={}; truth is not subscribed", backend, gpu_name, fuse ? "FUSED_RAW" : "RECORD_ONLY");
  }

  ~TunnelNode() override {
    for (const auto& pending : pending_calibration_ranges) log_range(pending.observation, "SHUTDOWN_PENDING", pending.bracket);
    for (const auto& observation : range_queue.reset()) log_range(observation, "SHUTDOWN_PENDING");
  }

  void report_failure(const std::string& detail) {
    {
      std::ofstream out;
      out.exceptions(std::ios::failbit | std::ios::badbit);
      out.open(RunLogManager::instance().runtime_path("odometry_failure.json"));
      out << nlohmann::json({{"state", "LOST"}, {"reason", failure_context}, {"detail", detail}, {"session", session}, {"measurement_stamp", latest_imu}}).dump(2) << '\n';
    }
    logger->error("{}: {}", failure_context, detail);
    status("LOST", failure_context, latest_imu);
  }

private:
  void reset() {
    gtsam::ISAM2Params p;
    p.relinearizeSkip = 1;
    p.relinearizeThreshold = cfg.at("odometry").value("relinearize_threshold", .05);
    if (fuse && (self_calibrate || cfg.at("uwb").value("continuous_installation_walls", false))) {
      if (!cfg.at("uwb").value("installation_factors", false) || (!self_calibrate && !lateral_reference)) throw std::invalid_argument("CONTINUOUS_WALLS_REQUIRE_INSTALLATION_REFERENCE");
      const double threshold = cfg.at("odometry").value("relinearize_threshold", .05);
      gtsam::FastMap<char, gtsam::Vector> thresholds;
      thresholds['x'] = gtsam::Vector::Constant(6, threshold);
      thresholds['v'] = gtsam::Vector::Constant(3, threshold);
      thresholds['b'] = gtsam::Vector::Constant(6, threshold);
      thresholds['a'] = gtsam::Vector::Constant(3, threshold);
      thresholds['l'] = gtsam::Vector2(1000 * threshold, threshold);
      p.relinearizeThreshold = thresholds;
    }
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
    for (const auto& pending : pending_calibration_ranges) log_range(pending.observation, "CLOCK_RESET", pending.bracket);
    pending_calibration_ranges.clear();
    for (const auto& pending : extension_ranges) log_range(pending.observation,"CLOCK_RESET",pending.bracket);
    extension_ranges.clear(); extension_batch.reset(); extension_previous.clear(); extension_last_attempt.clear();
    calibration_prepared = false;
    calibration_previous.reset();
    bootstrap_idle_failure = nullptr;
    calibration_solved.clear();
    if (self_calibrate) { map.ids = installation_input->ids; map.means.clear(); }
    bootstrap_factors.resize(0);
    bootstrap_values.clear();
    bootstrap_times.clear();
    stationary_seed_ranges.clear();
    bootstrap_branches.clear();
    bootstrap_batch.reset();
    bootstrap_last_attempt = -1;
    bootstrap_previous_winner.reset();
    initial_walls.reset();
    dedicated_geometry_cloud.reset();
    body_walls.reset();
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
    last_rejected = -1;
    uwb_recovery_until = -1;
    uwb_geometry_trace = std::numeric_limits<double>::infinity();
    postfit_ta_stamp = -1;
    failed_recovery_stamp = -1;
    postfit_ta_count = 0;
    postfit_ta_squared_sum = 0;
    rejected_total = 0;
    last_rejection.clear();
    last_status.clear();
    last_status_reason.clear();
    aa_summaries.clear();
    aa_added.clear();
    aa_removed.clear();
    aa_measurements = 0;
  }
  void status(const std::string& state, const std::string& reason, double t) {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus d;
    d.name = "liuo/odometry";
    d.hardware_id = backend;
    d.level = state == "TRACKING" ? 0 : (state == "LOST" ? 2 : 1);
    d.message = state + ":" + reason;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "measurement_stamp";
    kv.value = std::to_string(t);
    d.values.push_back(kv);
    kv.key = "rejected_total";
    kv.value = std::to_string(rejected_total);
    d.values.push_back(kv);
    kv.key = "last_rejection";
    kv.value = last_rejection;
    d.values.push_back(kv);
    kv.key = "uwb_recovery_until";
    kv.value = std::to_string(uwb_recovery_until);
    d.values.push_back(kv);
    kv.key = "failed_recovery_stamp";
    kv.value = std::to_string(failed_recovery_stamp);
    d.values.push_back(kv);
    kv.key = "postfit_ta_count";
    kv.value = std::to_string(postfit_ta_count);
    d.values.push_back(kv);
    kv.key = "postfit_ta_squared_sum";
    kv.value = std::to_string(postfit_ta_squared_sum);
    d.values.push_back(kv);
    kv.key = "uwb_optimistic_position_trace";
    kv.value = std::to_string(uwb_geometry_trace);
    d.values.push_back(kv);
    kv.key = "postfit_ta_limit";
    kv.value = std::to_string(postfit_range_limit(postfit_ta_count));
    d.values.push_back(kv);
    a.status.push_back(d);
    diagnostics->publish(a);
    if (state != last_status || reason != last_status_reason) {
      status_log << session << ',' << t << ',' << state << ',' << reason << ',' << rejected_total << ',' << last_rejection << ',' << uwb_recovery_until << '\n';
      status_log.flush();
      logger->info("state={} reason={} stamp={} session={}", state, reason, t, session);
      last_status = state;
      last_status_reason = reason;
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
    if (fuse && reason != "ACCEPT_TA" && reason != "ACCEPT_AA" && reason != "ORACLE_AA_CHECK" && reason != "CLOCK_RESET" && reason != "SHUTDOWN_PENDING" && reason != "PENDING_CALIBRATION" && reason != "AA_ABLATION_DISABLED" && reason != "ANCHOR_PENDING" && reason != "ANCHOR_WINDOW_EXPIRED") {
      ++rejected_total;
      last_rejected = now().seconds();
      last_rejection = reason;
    }
    uwb << o.producer << ',' << o.session << ',' << o.sequence << ',' << o.stamp << ',' << o.received << ',' << int(o.a_type) << ',' << o.a << ',' << int(o.b_type) << ',' << o.b
        << ',' << o.range << ',' << o.variance << ',' << reason << ',';
    if (bracket.reason == "SYNC" || bracket.reason == "INTERPOLATED" || bracket.reason == "STATIC_ALIGNMENT") uwb << gtsam::DefaultKeyFormatter(bracket.left);
    uwb << ',';
    if (bracket.right) uwb << gtsam::DefaultKeyFormatter(*bracket.right);
    uwb << ',' << anchor_key << ',' << category << ',' << residual << ',' << residual / std::sqrt(o.variance) << ",1,"
        << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << '\n';
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
      auto validate_id = [&](uint32_t id) {
        if (self_calibrate) {
          if (std::find(installation_input->ids.begin(), installation_input->ids.end(), id) == installation_input->ids.end())
            throw std::invalid_argument("UNKNOWN_ANCHOR");
        } else map.index(id);
      };
      validate_id(o.b);
      if (o.a_type == 1) validate_id(o.a);
    } catch (const std::exception&) {
      log_range(o, "UNKNOWN_ANCHOR");
      return;
    }
    auto reason = range_queue.push(o);
    if (reason != "QUEUED") log_range(o, reason);
    // Only already validated, deduplicated samples during the promised static
    // interval generate seeds. These summaries are NOT graph observations.
    if (reason == "QUEUED" && o.a_type == 0 && first_imu >= 0 && o.stamp >= first_imu && o.stamp <= first_imu + cfg.at("odometry").at("initialization_s").get<double>())
      stationary_seed_ranges[o.b].add(o.range, o.variance);
  }
  std::optional<gtsam::Values> solve_bootstrap_branches(double t, const gtsam_points::LevenbergMarquardtExtParams& params) {
    const double bootstrap_timeout = cfg.at("uwb").value("bootstrap_timeout_s", self_calibrate ? 60. : 20.);
    if (!std::isfinite(bootstrap_timeout) || bootstrap_timeout <= 0 || bootstrap_timeout > 60.) throw std::invalid_argument("BOOTSTRAP_TIMEOUT_MUST_BE_IN_0_60_SECONDS");
    if (t - first_imu >= bootstrap_timeout) {
      failure_context = "INITIALIZATION_AMBIGUOUS_TIMEOUT";
      throw std::runtime_error("finite bootstrap search did not certify a branch within " + std::to_string(bootstrap_timeout) + " s");
    }
    if (!bootstrap_batch && bootstrap_last_attempt >= 0 && t < bootstrap_last_attempt + 2. - 1e-8) return std::nullopt;
    if (!bootstrap_batch) bootstrap_last_attempt = t;
    auto& logs = RunLogManager::instance();
    std::ofstream branch_log(logs.profiling_path("bootstrap_branches.csv"), std::ios::app);
    branch_log << std::setprecision(16);
    if (branch_log.tellp() == 0) branch_log << "session,stamp,branch,cost,x0,y0,z0,yaw0,event,completion_stamp\n";
    if (bootstrap_branches.empty()) {
      // Pick the best conditioned available triangle, never anchor ID order as
      // a proxy for tunnel direction. Static means are proposal inputs only.
      std::vector<RangeSeedSphere> spheres;
      for (const auto& entry : stationary_seed_ranges)
        if (entry.second.count >= 8) spheres.push_back({map.means[map.index(entry.first)], entry.second.mean});
      double area = 0;
      std::array<RangeSeedSphere, 3> selected;
      for (size_t i = 0; i < spheres.size(); ++i)
        for (size_t j = i + 1; j < spheres.size(); ++j)
          for (size_t k = j + 1; k < spheres.size(); ++k) {
            const double candidate = (spheres[j].anchor - spheres[i].anchor).cross(spheres[k].anchor - spheres[i].anchor).norm();
            if (candidate > area) {
              area = candidate;
              selected = {spheres[i], spheres[j], spheres[k]};
            }
          }
      if (area == 0) {
        status("INITIALIZING", "INSUFFICIENT_SEED_GEOMETRY", t);
        return std::nullopt;
      }
      const auto tags = intersect_range_seeds(selected, reuse_map || cfg.at("uwb").value("nonintersecting_seed_proposals", false));
      logger->info("bootstrap seed strategy={} proposals={}", tags.reason, tags.tags.size());
      if (tags.tags.empty()) {
        status("INITIALIZING", tags.reason, t);
        return std::nullopt;
      }
      if (lateral_reference) {
        if (!initial_walls) {
          status("INITIALIZING", "LOCAL_WALLS_NOT_OBSERVED", t);
          return std::nullopt;
        }
        const auto seeds = lateral_pose_seeds(tags.tags, bootstrap_values.at<gtsam::Pose3>(X(0)).rotation(), tag, *initial_walls, *lateral_reference);
        for (const auto& seed : seeds) bootstrap_branches.push_back(transform_bootstrap_seed(bootstrap_values, X(0), seed));
      } else {
        const auto seeds = pose_seeds_from_tags(tags.tags, bootstrap_values.at<gtsam::Pose3>(X(0)).rotation(), tag);
        gtsam::NonlinearFactorGraph reference_graph;
        for (const auto& factor : bootstrap_factors) reference_graph.push_back(factor->clone());
        // This is a numerical reference trajectory, NOT a certified branch and
        // never a new pose observation. Every mirror/yaw seed is still tested.
        gtsam_points::LevenbergMarquardtOptimizerExt reference_optimizer(reference_graph, bootstrap_values, params);
        const auto reference = reference_optimizer.optimize();
        const auto reference_pose = reference.at<gtsam::Pose3>(X(0));
        branch_log << session << ',' << t << ",-1," << reference_optimizer.error() << ',' << reference_pose.x() << ',' << reference_pose.y() << ',' << reference_pose.z() << ','
                   << reference_pose.rotation().yaw() << ",REFERENCE_ONLY," << t << '\n';
        std::vector<gtsam::Values> proposals;
        for (size_t seed_id = 0; seed_id < seeds.size(); ++seed_id) {
          auto initial = transform_bootstrap_seed(reference, X(0), seeds[seed_id]);
          auto proposal = precondition_bootstrap(bootstrap_factors, initial);
          const auto pose = proposal.values.at<gtsam::Pose3>(X(0));
          branch_log << session << ',' << t << ',' << seed_id << ',' << proposal.cost << ',' << pose.x() << ',' << pose.y() << ',' << pose.z() << ',' << pose.rotation().yaw()
                     << ",RANGE_SEED_ONLY," << t << '\n';
          bool duplicate = false;
          for (const auto& retained : proposals) {
            const auto delta = retained.at<gtsam::Pose3>(X(0)).between(pose);
            if (delta.translation().norm() < .5 && gtsam::Rot3::Logmap(delta.rotation()).norm() < .1) {
              duplicate = true;
              break;
            }
          }
          if (!duplicate) proposals.push_back(std::move(proposal.values));
        }
        // Never certify uniqueness by truncating distinct competing basins.
        if (proposals.size() > 4) {
          status("INITIALIZING", "BOOTSTRAP_DISTINCT_BRANCH_BUDGET", t);
          return std::nullopt;
        }
        bootstrap_branches = std::move(proposals);
      }
    }
    if (!bootstrap_batch) {
      bootstrap_batch = std::make_unique<BootstrapBatch>();
      bootstrap_batch->stamp = t;
      bootstrap_batch->graph = bootstrap_factors;
      bootstrap_batch->initial = bootstrap_values;
      for (const auto& entry : aa_summaries) {
        const auto& stats = entry.second.statistics;
        bootstrap_batch->graph.emplace_shared<AnchorAnchorRangeFactor>(A(entry.first.first), A(entry.first.second), stats.mean, stats.sigma(), stats.scatter);
      }
    }
    auto& batch = *bootstrap_batch;
    const auto started = std::chrono::steady_clock::now();
    // Outer iterations are indivisible. The 30ms scheduling budget is soft,
    // not a claim that any single iteration always meets a hard deadline.
    do {
      const size_t i = batch.evidence.size();
      double cost = std::numeric_limits<double>::infinity();
      std::string event = "SOLVED";
      bool done = false;
      try {
        if (bootstrap_idle_failure) {
          auto failure = bootstrap_idle_failure;
          bootstrap_idle_failure = nullptr;
          std::rethrow_exception(failure);
        }
        if (!batch.optimizer) {
          const auto first = bootstrap_branches[i].at<gtsam::Pose3>(X(0));
          auto initial = transform_bootstrap_seed(batch.initial, X(0), first);
          for (auto key : bootstrap_branches[i].keys()) initial.update(key, bootstrap_branches[i].at(key));
          // Each candidate owns its nonlinear cache; GPU work stays serial.
          gtsam::NonlinearFactorGraph graph;
          for (const auto& factor : batch.graph) graph.push_back(factor->clone());
          if (self_calibrate) {
            batch.linearization_values = initial;
            if (calibration_solved.count(i) && cfg.at("uwb").value("bootstrap_joint_marginalization", false)) {
            const auto started = std::chrono::steady_clock::now();
            auto compact = marginalize_joint(graph, initial, calibration_elimination(initial, batch.stamp, true));
            std::ofstream marginal_log(logs.profiling_path("calibration_marginalization.csv"), std::ios::app);
            if (marginal_log.tellp() == 0) marginal_log << "snapshot_stamp,branch,source_factors,retired_factors,remaining_factors,remaining_values,milliseconds\n";
            marginal_log << batch.stamp << ',' << i << ',' << graph.size() << ',' << compact.retired_factors << ',' << compact.graph.size() << ',' << compact.values.size() << ','
                         << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count() << '\n';
            graph = std::move(compact.graph);
            initial = std::move(compact.values);
            }
          }
          batch.optimizer = std::make_unique<SteppedBootstrapLM>(graph, initial, params, bounded_bootstrap_work);
        }
        done = batch.optimizer->advance();
        if (done) {
          if (self_calibrate) {
            // Keep eliminated linearization points for this candidate's next
            // snapshot; they are not new observations or independent priors.
            bootstrap_branches[i] = batch.linearization_values;
            for (auto key : batch.optimizer->values().keys()) bootstrap_branches[i].update(key, batch.optimizer->values().at(key));
          } else bootstrap_branches[i] = batch.optimizer->values();
          cost = batch.optimizer->error();
          if (self_calibrate && std::isfinite(cost)) calibration_solved.insert(i);
        }
      } catch (const std::exception& error) {
        done = true;
        event = "SOLVE_FAILED";
        logger->warn("bootstrap branch {} failed: {}", i, error.what());
      }
      if (done) {
        const auto pose = bootstrap_branches[i].at<gtsam::Pose3>(X(0));
        batch.evidence.push_back({cost, pose});
        branch_log << session << ',' << batch.stamp << ',' << i << ',' << cost << ',' << pose.x() << ',' << pose.y() << ',' << pose.z() << ',' << pose.rotation().yaw() << ','
                   << event << ',' << t << '\n';
        if (self_calibrate) {
          std::ofstream anchor_log(logs.profiling_path("calibration_branches.csv"), std::ios::app);
          if (anchor_log.tellp() == 0) anchor_log << "session,snapshot_stamp,completion_stamp,branch,anchor,x,y,z,cost,event\n";
          anchor_log << std::setprecision(16);
          for (auto id : map.ids) {
            const auto point = bootstrap_branches[i].at<gtsam::Point3>(A(id));
            anchor_log << session << ',' << batch.stamp << ',' << t << ',' << i << ',' << id << ',' << point.x() << ',' << point.y() << ',' << point.z() << ',' << cost << ',' << event << '\n';
          }
        }
        batch.optimizer.reset();
      }
    } while (batch.evidence.size() < bootstrap_branches.size() && std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count() < 30.);
    std::ofstream work_log(logs.profiling_path("bootstrap_work.csv"), std::ios::app);
    if (work_log.tellp() == 0) work_log << "session,stamp,snapshot_stamp,finished_candidates,work_ms,current_factors,snapshot_factors\n";
    work_log << std::setprecision(16) << session << ',' << t << ',' << batch.stamp << ',' << batch.evidence.size() << ','
             << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count() << ',' << bootstrap_factors.size() << ',' << batch.graph.size()
             << '\n';
    if (batch.evidence.size() < bootstrap_branches.size()) {
      status("INITIALIZING", "CANDIDATE_OPTIMIZATION_IN_PROGRESS", t);
      return std::nullopt;
    }
    const auto evidence = std::move(batch.evidence);
    bootstrap_batch.reset();
    const size_t winner = std::distance(evidence.begin(), std::min_element(evidence.begin(), evidence.end(), [](const auto& a, const auto& b) { return a.cost < b.cost; }));
    bool unresolved_position = unresolved_position_branch(evidence, winner);
    bool separated = !unresolved_position && separated_bootstrap_branch(evidence, winner);
    if (self_calibrate) {
      unresolved_position = false;
      separated = std::isfinite(evidence[winner].cost);
      for (size_t i = 0; i < evidence.size(); ++i) {
        if (!std::isfinite(evidence[i].cost)) { separated = false; continue; }
        double distance = 0;
        for (auto id : map.ids) distance = std::max(distance, (bootstrap_branches[i].at<gtsam::Point3>(A(id)) - bootstrap_branches[winner].at<gtsam::Point3>(A(id))).norm());
        if (distance > .5 && evidence[i].cost < evidence[winner].cost + 5.) unresolved_position = true;
      }
      separated = separated && !unresolved_position;
    }
    bool stable = false;
    if (separated && bootstrap_previous_winner) {
      const auto delta = bootstrap_previous_winner->between(evidence[winner].first_pose);
      stable = delta.translation().norm() < .5 && gtsam::Rot3::Logmap(delta.rotation()).norm() < .1;
    }
    if (self_calibrate) {
      stable = separated && calibration_previous.has_value();
      if (stable)
        for (auto id : map.ids)
          if ((calibration_previous->at<gtsam::Point3>(A(id)) - bootstrap_branches[winner].at<gtsam::Point3>(A(id))).norm() >= .5) stable = false;
      calibration_previous = separated ? std::optional<gtsam::Values>(bootstrap_branches[winner]) : std::nullopt;
    }
    bootstrap_previous_winner = separated ? std::optional<gtsam::Pose3>(evidence[winner].first_pose) : std::nullopt;
    if (!stable) {
      status("INITIALIZING", unresolved_position ? "POSITION_BRANCH_UNVERIFIED" : (separated ? "BRANCH_AWAITING_NEW_DATA" : "BOOTSTRAP_BRANCH_AMBIGUITY"), t);
      return std::nullopt;
    }
    // The comparison used one frozen graph. Transfer ALL observations arriving
    // during its solve too, with the current AA summary exactly once. Those new
    // states are initial guesses, refined by the same original joint graph.
    auto result = transform_bootstrap_seed(bootstrap_values, X(0), bootstrap_branches[winner].at<gtsam::Pose3>(X(0)));
    for (auto key : bootstrap_branches[winner].keys()) result.update(key, bootstrap_branches[winner].at(key));
    aa_added.clear();
    for (const auto& entry : aa_summaries) {
      aa_added.emplace_back(entry.first, bootstrap_factors.size());
      const auto& stats = entry.second.statistics;
      bootstrap_factors.emplace_shared<AnchorAnchorRangeFactor>(A(entry.first.first), A(entry.first.second), stats.mean, stats.sigma(), stats.scatter);
    }
    return result;
  }
  gtsam::Ordering calibration_elimination(const gtsam::Values& values, double stamp, bool keep_origin) const {
    gtsam::Ordering keys;
    const double cutoff = stamp - cfg.at("odometry").at("lag_s").get<double>();
    for (auto key : values.keys()) {
      const auto symbol = gtsam::Symbol(key);
      if (symbol.chr() != 'x' && symbol.chr() != 'v' && symbol.chr() != 'b') continue;
      if (keep_origin && key == X(0)) continue;
      const auto found = bootstrap_times.find(key);
      if (found == bootstrap_times.end()) throw std::logic_error("CALIBRATION_TIMESTAMP_MISSING");
      if (found->second < cutoff) keys.push_back(key);
    }
    return keys;
  }

  void export_calibrated_map(double stamp) {
    if (!self_calibrate || !bootstrapped) return;
    const auto values = smoother->calculateEstimate();
    gtsam::KeyVector keys;
    nlohmann::json anchors = nlohmann::json::array(), missing = nlohmann::json::array();
    for (auto id : map.ids) {
      keys.push_back(A(id));
      const auto a = values.at<gtsam::Point3>(A(id));
      anchors.push_back({{"id", id}, {"mean_m", {a.x(), a.y(), a.z()}}});
    }
    keys.push_back(gtsam::Symbol('l', 0));
    const auto line = values.at<gtsam::Vector2>(keys.back());
    const auto covariance = selected_covariance(smoother->getISAM2(), keys);
    std::vector<std::vector<double>> matrix(covariance.rows(), std::vector<double>(covariance.cols()));
    for (int i = 0; i < covariance.rows(); ++i)
      for (int j = 0; j < covariance.cols(); ++j) matrix[i][j] = covariance(i,j);
    gtsam::Point3 lower = gtsam::Point3::Constant(std::numeric_limits<double>::infinity());
    gtsam::Point3 upper = -lower;
    for (auto key : values.keys()) if (gtsam::Symbol(key).chr() == 'x') {
      const auto p = values.at<gtsam::Pose3>(key).translation();
      lower = lower.cwiseMin(p); upper = upper.cwiseMax(p);
    }
    for (auto id : installation_input->ids)
      if (std::find(map.ids.begin(), map.ids.end(), id) == map.ids.end())
        missing.push_back({{"id", id}, {"reason", "NOT_JOINTLY_INITIALIZED"}});
    auto& logs = RunLogManager::instance();
    std::ifstream identity_file(logs.run_dir() / "metadata/manifests/map_source_identity.json");
    if (!identity_file) throw std::runtime_error("MAP_EXPORT_INPUT_IDENTITY_MISSING");
    const auto identity = nlohmann::json::parse(identity_file);
    nlohmann::json document = {
      {"schema_version", "liuo_joint_map_v1"}, {"frame", "map"}, {"anchor_units", "m"},
      {"reference_units", {"mrad", "m"}}, {"reference_origin_m", {0,0,0}},
      {"map_id", logs.run_dir().filename().string()+"_session_"+std::to_string(session)},
      {"coordinate_gauge", "first IMU gravity alignment; initial translation zero; yaw is coordinate convention"},
      {"status", "PROVISIONAL_INITIALIZED"}, {"deployment_validated", false},
      {"unresolved_branches", nlohmann::json::array()},
      {"branch_scope", "finite candidate search only; not a global uniqueness certificate"},
      {"sensor_identity", cfg.at("sensors")}, {"source_run", logs.run_dir().string()},
      {"source_identity", identity}, {"measurement_stamp", stamp}, {"session", session},
      {"approximation", "Joint A/L Gaussian marginal of current fixed-lag Bayes tree, centered at current estimate; local linearization and single-basin approximation; statistical consistency not certified"},
      {"installation_provenance", {{"input", installation_input->source}, {"already_absorbed", true}}},
      {"not_initialized", missing}, {"anchors", anchors}, {"reference_mean", {line[0],line[1]}},
      {"joint_covariance", matrix},
      {"estimated_vehicle_bounds_m", {{lower.x(),lower.y(),lower.z()},{upper.x(),upper.y(),upper.z()}}},
      {"bounds_scope", "current retained vehicle states only; not a certified reuse region"},
      {"absorbed_information", "all accepted first-session raw factors up to measurement_stamp; do not reload installation or first-run ranges as independent observations"}
    };
    CalibratedMap::parse(document, cfg.at("sensors")).save_atomic(logs.export_path("map/calibrated_map.json"));
    map_export_session = session;
    map_export_anchor_count = map.ids.size();
  }
  bool variable_anchors() const { return reuse_map || map.surveyed() || (self_calibrate && calibration_prepared); }

  bool prepare_calibration(double t) {
    if (!body_walls) return false;
    std::map<uint32_t, std::vector<TagRangeSeedObservation>> samples;
    for (const auto& pending : pending_calibration_ranges) {
      const auto& o = pending.observation;
      if (o.a_type != 0) continue;
      const auto& b = pending.bracket;
      const auto left = bootstrap_values.at<gtsam::Pose3>(b.left);
      const auto point = b.right ? interpolated_tag(left, bootstrap_values.at<gtsam::Pose3>(*b.right), b.beta, tag) : left.transformFrom(tag);
      samples[o.b].push_back({point, o.range});
    }
    if (samples.size() < 3) return false;
    const auto pose = bootstrap_values.at<gtsam::Pose3>(X(0));
    const auto world_normal = pose.rotation().rotate(body_walls->normal);
    const Eigen::Vector2d normal = world_normal.head<2>().normalized();
    const auto center = pose.transformFrom(body_walls->center);
    std::vector<uint32_t> ids;
    std::vector<size_t> installation_rows;
    for (const auto& entry : samples) {
      ids.push_back(entry.first);
      installation_rows.push_back(std::distance(installation_input->ids.begin(), std::find(installation_input->ids.begin(), installation_input->ids.end(), entry.first)));
    }
    std::vector<gtsam::Values> proposals;
    for (double sign : {-1., 1.}) {
      const Eigen::Vector2d n = sign * normal;
      const double offset = n.dot(center.head<2>());
      std::vector<gtsam::Values> side{bootstrap_values};
      side.front().insert(gtsam::Symbol('l', 0), gtsam::Vector2(1000 * std::atan2(n.y(), n.x()), offset));
      for (size_t i = 0; i < ids.size(); ++i) {
        const auto& all = samples.at(ids[i]);
        std::vector<TagRangeSeedObservation> selected;
        // Decimation affects proposals only. ALL raw ranges enter the graph.
        const size_t count = std::min(size_t(512), all.size());
        if (count < 8) return false;
        for (size_t k = 0; k < count; ++k) selected.push_back(all[k * (all.size() - 1) / (count - 1)]);
        auto seeds = unknown_anchor_seeds(selected, n, offset, installation_input->offsets[installation_rows[i]]);
        if (seeds.points.empty()) return false;
        // Tiny estimated vertical motion is not proof that a mirror is gone.
        if (seeds.points.size() == 1) {
          auto mirror = seeds.points.front();
          double height = 0;
          for (const auto& sample : selected) height += sample.tag.z();
          mirror.z() = 2 * height / selected.size() - mirror.z();
          seeds.points.push_back(mirror);
        }
        if (side.size() * seeds.points.size() + proposals.size() > 32) {
          status("INITIALIZING", "UNKNOWN_ANCHOR_BRANCH_CAPACITY", t);
          return false;
        }
        std::vector<gtsam::Values> expanded;
        for (const auto& candidate : side)
          for (const auto& point : seeds.points) {
            auto next = candidate;
            next.insert(A(ids[i]), point);
            expanded.push_back(std::move(next));
          }
        side = std::move(expanded);
      }
      proposals.insert(proposals.end(), side.begin(), side.end());
    }
    map.ids = ids;
    map.means.clear();
    gtsam::KeyVector keys;
    gtsam::Vector offsets(ids.size());
    gtsam::Matrix covariance(ids.size(), ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      keys.push_back(A(ids[i]));
      map.means.push_back(proposals.front().at<gtsam::Point3>(keys.back()));
      offsets[i] = installation_input->offsets[installation_rows[i]];
      for (size_t j = 0; j < ids.size(); ++j) covariance(i, j) = installation_input->covariance(installation_rows[i], installation_rows[j]);
      bootstrap_values.insert(keys.back(), map.means.back());
      bootstrap_times[keys.back()] = t;
    }
    const auto line = gtsam::Symbol('l', 0);
    bootstrap_values.insert(line, proposals.front().at<gtsam::Vector2>(line));
    bootstrap_times[line] = t;
    bootstrap_factors.emplace_shared<InstallationOffsetFactor>(line, keys, offsets, covariance, gtsam::Point3::Zero(), 1000.);
    bootstrap_factors.emplace_shared<WallReferenceFactor>(X(0), line, body_walls->normal, body_walls->center, body_walls->covariance, gtsam::Point3::Zero(), 1000.);
    for (const auto& pending : pending_calibration_ranges) {
      const auto& o = pending.observation;
      const auto& b = pending.bracket;
      if (o.a_type == 0) {
        bootstrap_factors.emplace_shared<TagAnchorRangeFactor>(b.left, b.right, A(o.b), gtsam::Point3::Zero(), tag, b.beta, o.range, std::sqrt(o.variance));
        last_uwb = std::max(last_uwb, o.stamp);
        log_range(o, "ACCEPT_TA", b, gtsam::DefaultKeyFormatter(A(o.b)), NAN, "CALIBRATION_RAW_TRANSFER");
      } else if (std::find(ids.begin(), ids.end(), o.a) != ids.end() && std::find(ids.begin(), ids.end(), o.b) != ids.end()) {
        const auto pair = std::make_pair(std::min(o.a, o.b), std::max(o.a, o.b));
        aa_summaries[pair].statistics.add(o.range, o.variance);
        ++aa_measurements;
        log_range(o, "ACCEPT_AA", {}, "", NAN, "CALIBRATION_RAW_TRANSFER");
      } else {
        log_range(o, "AA_ANCHOR_PENDING");
      }
    }
    pending_calibration_ranges.clear();
    bootstrap_branches = std::move(proposals);
    calibration_prepared = true;
    return true;
  }

  void advance_anchor_extension(double t, gtsam::NonlinearFactorGraph& frame_factors) {
    if (!bootstrapped || extension_ranges.empty()) return;
    const double lag = cfg.at("odometry").at("lag_s");
    auto current = smoother->calculateEstimate();
    auto known = [&](uint32_t id) { return current.exists(A(id)); };
    auto usable = [&](const PendingCalibrationRange& p) {
      return p.observation.stamp >= t - lag && (p.observation.a_type == 1 ||
        (current.exists(p.bracket.left) && (!p.bracket.right || current.exists(*p.bracket.right))));
    };
    for (auto it = extension_ranges.begin(); it != extension_ranges.end();) {
      if (!usable(*it)) { log_range(it->observation,"ANCHOR_WINDOW_EXPIRED",it->bracket); it = extension_ranges.erase(it); }
      else ++it;
    }
    auto log = [&](uint32_t id, const char* event, double a = NAN, double b = NAN) {
      std::ofstream out(RunLogManager::instance().profiling_path("anchor_extension.csv"),std::ios::app);
      if (out.tellp() == 0) out << "session,stamp,anchor,event,value_a,value_b,pending_records\n";
      out << std::setprecision(16) << session << ',' << t << ',' << id << ',' << event << ',' << a << ',' << b << ',' << extension_ranges.size() << '\n';
    };
    if (extension_batch && extension_batch->oldest < t-lag) {
      log(extension_batch->id,"SNAPSHOT_WINDOW_EXPIRED"); extension_batch.reset();
    }
    if (!extension_batch) {
      std::map<uint32_t,std::vector<TagRangeSeedObservation>> samples;
      for (const auto& p : extension_ranges) if (p.observation.a_type == 0 && p.observation.stamp >= t-lag/2) {
        const auto& b=p.bracket;
        const auto left=current.at<gtsam::Pose3>(b.left);
        const auto point=b.right ? interpolated_tag(left,current.at<gtsam::Pose3>(*b.right),b.beta,tag) : left.transformFrom(tag);
        samples[p.observation.b].push_back({point,p.observation.range});
      }
      const auto line=current.at<gtsam::Vector2>(gtsam::Symbol('l',0));
      const Eigen::Vector2d normal(std::cos(line[0]/1000.),std::sin(line[0]/1000.));
      for (const auto& entry : samples) {
        const auto id=entry.first;
        if (extension_last_attempt.count(id) && t-extension_last_attempt.at(id)<2.) continue;
        const auto row=std::distance(installation_input->ids.begin(),std::find(installation_input->ids.begin(),installation_input->ids.end(),id));
        auto seeds=unknown_anchor_seeds(entry.second,normal,line[1],installation_input->offsets[row]);
        if (seeds.points.empty()) continue;
        if (seeds.points.size()==1) {
          auto mirror=seeds.points.front(); double height=0;
          for (const auto& p : entry.second) height+=p.tag.z();
          mirror.z()=2*height/entry.second.size()-mirror.z(); seeds.points.push_back(mirror);
        }
        extension_batch=std::make_unique<ExtensionBatch>();
        auto& batch=*extension_batch;
        batch.id=id; batch.stamp=t; batch.oldest=t; batch.initial=current; batch.seeds=seeds.points;
        const auto& old=smoother->getFactors();
        const size_t replaced=installation_subset_index(old,map.ids);
        for (size_t i=0;i<old.size();++i) if (old[i] && i!=replaced) batch.graph.push_back(old[i]->clone());
        auto ids=map.ids; ids.push_back(id);
        batch.graph.add(joint_installation_factor(*installation_input,ids));
        for (const auto& p : extension_ranges) {
          const auto& o=p.observation; const auto& b=p.bracket;
          if (o.stamp<t-lag/2) continue;
          if (o.a_type==0 && o.b==id) {
            batch.graph.emplace_shared<TagAnchorRangeFactor>(b.left,b.right,A(id),gtsam::Point3::Zero(),tag,b.beta,o.range,std::sqrt(o.variance));
          } else if (o.a_type==1 && ((o.a==id && known(o.b)) || (o.b==id && known(o.a)))) {
            batch.graph.emplace_shared<AnchorAnchorRangeFactor>(A(o.a),A(o.b),o.range,std::sqrt(o.variance));
          } else continue;
          batch.oldest=std::min(batch.oldest,o.stamp);
        }
        extension_last_attempt[id]=t;
        log(id,"CANDIDATES_STARTED",seeds.baseline,batch.graph.size());
        break;
      }
      if (!extension_batch) return;
    }
    auto& batch=*extension_batch;
    if (!batch.optimizer) {
      auto initial=batch.initial;
      initial.insert(A(batch.id),batch.seeds.at(batch.solutions.size()));
      gtsam::NonlinearFactorGraph graph;
      for (const auto& f : batch.graph) graph.push_back(f->clone());
      gtsam_points::LevenbergMarquardtExtParams params;
      params.maxIterations=cfg.at("uwb").value("bootstrap_max_iterations",30);
      // Existing marginalized/raw factors carry a large background cost. A
      // relative TOTAL-cost test can stop before the new anchor is optimized.
      // Keep absolute convergence and the iteration/window budgets unchanged.
      params.relativeErrorTol=0;
      batch.optimizer=std::make_unique<SteppedBootstrapLM>(graph,initial,params,true);
    }
    try {
      if (batch.failure) std::rethrow_exception(batch.failure);
      if (!batch.optimizer->advance()) return;
      const auto solution=batch.optimizer->values().at<gtsam::Point3>(A(batch.id));
      const double cost=batch.optimizer->error();
      if (!solution.allFinite() || !std::isfinite(cost)) throw std::runtime_error("NONFINITE_ANCHOR_CANDIDATE");
      log(batch.id,"CANDIDATE_ITERATIONS",batch.optimizer->iterations(),cost);
      batch.solutions.push_back(solution); batch.costs.push_back(cost); batch.optimizer.reset();
      log(batch.id,"CANDIDATE_SOLVED",solution.z(),cost);
    } catch (const std::exception& error) {
      logger->warn("anchor {} candidate failed: {}",batch.id,error.what());
      log(batch.id,"CANDIDATE_FAILED"); extension_previous.erase(batch.id); extension_batch.reset(); return;
    }
    if (batch.solutions.size()<batch.seeds.size()) return;
    const uint32_t id=batch.id;
    const size_t best=std::distance(batch.costs.begin(),std::min_element(batch.costs.begin(),batch.costs.end()));
    const auto point=batch.solutions[best];
    for (const auto& alternative : batch.solutions) if ((alternative-point).norm()>=.5) {
      log(id,"UNRESOLVED_POSITION_BRANCH",(alternative-point).norm());
      extension_previous.erase(id); extension_batch.reset(); return;
    }
    const bool stable=extension_previous.count(id) && (extension_previous.at(id)-point).norm()<.5;
    extension_previous.insert_or_assign(id,point);
    if (!stable) { log(id,"AWAITING_NEW_DATA"); extension_batch.reset(); return; }
    // Commit only current-window RAW measurements. Candidate robot states and
    // candidate covariance never become independent observations or priors.
    const size_t old_installation=installation_subset_index(smoother->getFactors(),map.ids);
    auto ids=map.ids; ids.push_back(id);
    gtsam::NonlinearFactorGraph additions;
    additions.add(joint_installation_factor(*installation_input,ids));
    gtsam::Values new_values; new_values.insert(A(id),point);
    std::map<std::pair<uint32_t,uint32_t>,GaussianRangeSummary> new_pairs;
    size_t accepted_ta=0,accepted_aa=0;
    for (auto it=extension_ranges.begin();it!=extension_ranges.end();) {
      const auto& o=it->observation; const auto& b=it->bracket;
      if (o.a_type==0 && o.b==id) {
        additions.emplace_shared<TagAnchorRangeFactor>(b.left,b.right,A(id),gtsam::Point3::Zero(),tag,b.beta,o.range,std::sqrt(o.variance));
        log_range(o,"ACCEPT_TA",b,gtsam::DefaultKeyFormatter(A(id)),NAN,"DELAYED_RAW_TRANSFER"); ++accepted_ta;
      } else if (o.a_type==1 && ((o.a==id && known(o.b)) || (o.b==id && known(o.a)))) {
        new_pairs[{std::min(o.a,o.b),std::max(o.a,o.b)}].add(o.range,o.variance);
        log_range(o,"ACCEPT_AA",{},"",NAN,"DELAYED_RAW_TRANSFER"); ++accepted_aa;
      } else { ++it; continue; }
      it=extension_ranges.erase(it);
    }
    if (!accepted_ta) throw std::logic_error("EXTENSION_HAS_NO_CURRENT_RAW_TA");
    std::vector<std::pair<std::pair<uint32_t,uint32_t>,size_t>> pair_indices;
    for (const auto& entry : new_pairs) {
      if (aa_summaries.count(entry.first)) throw std::logic_error("EXTENSION_AA_ALREADY_ABSORBED");
      pair_indices.emplace_back(entry.first,additions.size());
      const auto& stats=entry.second;
      additions.emplace_shared<AnchorAnchorRangeFactor>(A(entry.first.first),A(entry.first.second),stats.mean,stats.sigma(),stats.scatter);
    }
    smoother->update(additions,new_values,{{A(id),t}},{old_installation});
    for (const auto& entry : pair_indices) {
      auto& summary=aa_summaries[entry.first]; summary.statistics=new_pairs.at(entry.first);
      summary.factor_index=smoother->getISAM2Result().newFactorsIndices.at(entry.second);
    }
    aa_measurements+=accepted_aa; new_aa+=accepted_aa; new_ta+=accepted_ta;
    frame_factors.add(additions);
    map.ids=std::move(ids); map.means.push_back(point);
    log(id,"JOINTLY_INITIALIZED",accepted_ta,accepted_aa);
    extension_previous.erase(id); extension_batch.reset();
  }

  void retain_calibration_ranges(double t) {
    std::map<double, gtsam::Key> poses;
    for (const auto& frame : frames) poses[frame.t] = X(frame.id);
    poses[t] = X(count);
    for (const auto& observation : range_queue.take_until(t)) {
      if (observation.a_type == 1 && !cfg.at("uwb").value("use_aa", true)) { log_range(observation, "AA_ABLATION_DISABLED"); continue; }
      auto bracket = count == 0
          ? static_alignment_bracket(poses, observation.stamp, first_imu, first_imu + cfg.at("odometry").at("initialization_s").get<double>())
          : range_bracket(poses, observation.stamp);
      if (observation.a_type == 0 && bracket.reason != "SYNC" && bracket.reason != "INTERPOLATED" && bracket.reason != "STATIC_ALIGNMENT") {
        log_range(observation, bracket.reason);
        continue;
      }
      if (pending_calibration_ranges.size() >= 12000) throw std::runtime_error("SELF_CALIBRATION_RANGE_CAPACITY");
      pending_calibration_ranges.push_back({observation, bracket});
      // Raw identities have already passed the queue's deduplication. Do not
      // push them back through that queue or treat this retention as fusion.
      log_range(observation, "PENDING_CALIBRATION", bracket);
    }
  }
  void append_ranges(double t, gtsam::NonlinearFactorGraph& factors, const gtsam::Values& predicted) {
    new_ta = 0;
    new_aa = 0;
    aa_added.clear();
    aa_removed.clear();
    std::set<std::pair<uint32_t, uint32_t>> touched_aa;
    std::map<double, gtsam::Key> poses;
    for (const auto& frame : frames) poses[frame.t] = X(frame.id);
    poses[t] = X(count);
    for (const auto& o : range_queue.take_until(t)) {
      if (o.a_type == 1 && !cfg.at("uwb").value("use_aa", true)) { log_range(o, "AA_ABLATION_DISABLED"); continue; }
      auto bracket = count == 0 && cfg.at("uwb").value("use_static_alignment_ranges", false)
                       ? static_alignment_bracket(poses, o.stamp, first_imu, first_imu + cfg.at("odometry").at("initialization_s").get<double>())
                       : range_bracket(poses, o.stamp);
      if (bracket.reason != "SYNC" && bracket.reason != "INTERPOLATED" && bracket.reason != "STATIC_ALIGNMENT") {
        log_range(o, bracket.reason);
        continue;
      }
      if (self_calibrate && (std::find(map.ids.begin(),map.ids.end(),o.b) == map.ids.end() ||
          (o.a_type == 1 && std::find(map.ids.begin(),map.ids.end(),o.a) == map.ids.end()))) {
        if (extension_ranges.size() >= 6000) throw std::runtime_error("ANCHOR_PENDING_RANGE_CAPACITY");
        extension_ranges.push_back({o,bracket});
        log_range(o,"ANCHOR_PENDING",bracket);
        continue;
      }
      const size_t j = map.index(o.b);
      if (o.a_type == 1) {
        const size_t i = map.index(o.a);
        if (!variable_anchors()) {
          log_range(o, "ORACLE_AA_CHECK", {}, "", o.range - (map.means[i] - map.means[j]).norm(), "CONSISTENCY_ONLY");
          continue;
        }
        auto f = gtsam::make_shared<AnchorAnchorRangeFactor>(A(o.a), A(o.b), o.range, std::sqrt(o.variance));
        auto pair = std::make_pair(std::min(o.a, o.b), std::max(o.a, o.b));
        aa_summaries[pair].statistics.add(o.range, o.variance);
        touched_aa.insert(pair);
        ++aa_measurements;
        new_aa++;
        log_range(o, "ACCEPT_AA", {}, gtsam::DefaultKeyFormatter(A(o.a)) + ";" + gtsam::DefaultKeyFormatter(A(o.b)), f->unwhitenedError(predicted)[0], "PRE_OPT_INNOVATION");
      } else {
        auto anchor = variable_anchors() ? std::optional<gtsam::Key>(A(o.b)) : std::nullopt;
        auto f = gtsam::make_shared<TagAnchorRangeFactor>(bracket.left, bracket.right, anchor, map.means[j], tag, bracket.beta, o.range, std::sqrt(o.variance));
        double residual = f->unwhitenedError(predicted)[0];
        factors.add(f);
        new_ta++;
        // Fresh input after a real measurement-time gap does not instantly
        // make the dead-reckoning window healthy. Require one full lag of
        // renewed input before returning to TRACKING; this never alters X/V/B.
        if (bootstrapped && last_uwb >= 0 && o.stamp > last_uwb + .3) uwb_recovery_until = std::max(uwb_recovery_until, t + cfg.at("odometry").at("lag_s").get<double>());
        last_uwb = std::max(last_uwb, o.stamp);
        log_range(
          o,
          "ACCEPT_TA",
          bracket,
          anchor ? gtsam::DefaultKeyFormatter(*anchor) : "constant:" + std::to_string(o.b),
          residual,
          bracket.reason == "STATIC_ALIGNMENT" ? "STATIC_ALIGNMENT_PRE_OPT_INNOVATION" : "PRE_OPT_INNOVATION");
      }
    }
    if (bootstrapped) {
      for (const auto& pair : touched_aa) {
        auto& summary = aa_summaries.at(pair);
        if (summary.factor_index) aa_removed.push_back(*summary.factor_index);
        aa_added.emplace_back(pair, factors.size());
        const auto& stats = summary.statistics;
        factors.emplace_shared<AnchorAnchorRangeFactor>(A(pair.first), A(pair.second), stats.mean, stats.sigma(), stats.scatter);
      }
    }
  }
  void bind_aa_factor_indices() {
    const auto& indices = smoother->getISAM2Result().newFactorsIndices;
    for (const auto& entry : aa_added) aa_summaries.at(entry.first).factor_index = indices.at(entry.second);
    aa_added.clear();
    aa_removed.clear();
  }
  void drain() {
    if (clouds.empty() && extension_batch && extension_batch->optimizer && !extension_batch->optimizer->finished() && !extension_batch->failure) {
      try { extension_batch->optimizer->advance(); }
      catch (...) { extension_batch->failure = std::current_exception(); }
      return;
    }
    if (clouds.empty() && bounded_bootstrap_work && bootstrap_batch && bootstrap_batch->optimizer &&
        !bootstrap_batch->optimizer->finished() && !bootstrap_idle_failure) {
      // One original LM damping attempt on the existing frozen snapshot.
      // No graph mutation, new observations or second initialization pathway.
      try { bootstrap_batch->optimizer->advance(); }
      catch (...) { bootstrap_idle_failure = std::current_exception(); }
      return;
    }
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
        if (cfg.at("uwb").value("installation_factors", false)) dedicated_geometry_cloud = cloud;
        status("INITIALIZING", "IMU_STATIC_ALIGNMENT", t);
        continue;
      }
      failure_context = "PREPROCESS_FAILED";
      process(*cloud, t);
      failure_context = "SENSOR_INPUT_FAILED";
      // Return to the single executor between graph updates so pending raw
      // sensor callbacks can enqueue. Retain every remaining cloud in order.
      return;
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
    const double line_angle_scale = (self_calibrate || reuse_map || cfg.at("uwb").value("continuous_installation_walls", false)) ? 1000.0 : 1.0;
    const bool ongoing_walls = (bootstrapped || self_calibrate) && fuse && body_walls && cfg.at("uwb").value("continuous_installation_walls", false) && count % 10 == 0;
    std::vector<gtsam::Vector3> wall_points;
    size_t raw_index = 0;
    std::set<std::tuple<int, int, int>> occupied;
    double leaf = cfg.at("odometry").at("downsample_m");
    sensor_msgs::PointCloud2ConstIterator<float> x(m, "x"), y(m, "y"), z(m, "z"), dt(m, "time"), intensity(m, "intensity");
    for (; x != x.end(); ++x, ++y, ++z, ++dt, ++intensity, ++raw_index) {
      if (*dt != 0) throw std::runtime_error("nonzero per-point time with snapshot model; rolling not implemented");
      Eigen::Vector3d q(*x, *y, *z);
      if (!q.allFinite()) continue;
      q = T_BL * q;
      if (dedicated_wall_point(ongoing_walls, raw_index)) {
        wall_points.push_back(q);
        continue;
      }
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
    const double tangent_variance = cfg.at("odometry").value("cloud_tangent_variance", 1.0);
    CloudCovarianceEstimation covariance(2, cfg.at("odometry").at("validate_covariance_planarity"), tangent_variance);
    auto covariances = covariance.estimate(points, neighbors);
    Eigen::Vector3d surface_spectrum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d translation_projector = Eigen::Matrix3d::Identity();
    if (cfg.at("odometry").value("project_weak_lidar_translation", false)) translation_projector = surface_translation_projector(covariances, &surface_spectrum);
    size_t nonplanar = 0;
    for (const auto& c : covariances)
      if (c.trace() > 2.9 * tangent_variance) nonplanar++;
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
    const double lidar_weight = cfg.at("odometry").value("normalize_lidar_by_point_count", false) ? 1.0 / points.size() : 1.0;
    auto preprocessed = std::chrono::steady_clock::now();
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph factors, lidar;
    if (!initialized) {
      failure_context = "INITIALIZATION_FAILED";
      Eigen::Vector3d avg = sum_a / initial_samples;
      if (std::abs(avg.norm() - 9.80665) > .3 || (sum_w / initial_samples).norm() > .1) throw std::runtime_error("initialization requires stationary IMU interval");
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(avg, Eigen::Vector3d(0, 0, 9.80665));
      if (lateral_reference || self_calibrate) {
        std::vector<Eigen::Vector3d> leveled;
        if (cfg.at("uwb").value("installation_factors", false)) {
          if (!dedicated_geometry_cloud || (!self_calibrate && !variable_anchors())) throw std::runtime_error("DEDICATED_GEOMETRY_OR_SURVEYED_MAP_MISSING");
          std::vector<gtsam::Vector3> dedicated;
          const auto& observed = *dedicated_geometry_cloud;
          if (observed.header.frame_id != "lidar" || observed.is_bigendian) throw std::runtime_error("DEDICATED_GEOMETRY_FRAME_INVALID");
          sensor_msgs::PointCloud2ConstIterator<float> gx(observed, "x"), gy(observed, "y"), gz(observed, "z");
          for (; gx != gx.end(); ++gx, ++gy, ++gz) {
            const gtsam::Vector3 p(*gx, *gy, *gz);
            if (p.allFinite()) dedicated.push_back(T_BL * p);
          }
          body_walls = body_wall_observation(dedicated, gtsam::Rot3(q), cfg.at("sensors").at("sigma_lidar_range"));
          initial_walls = body_walls->leveled;
          std::ofstream evidence(RunLogManager::instance().profiling_path("dedicated_wall_observation.json"));
          evidence << nlohmann::json(
                        {{"cloud_stamp", rclcpp::Time(observed.header.stamp).seconds()},
                         {"pose_stamp", t},
                         {"points", dedicated.size()},
                         {"used_by_vgicp", false},
                         {"static_interval_assumption", true},
                         {"body_normal", {body_walls->normal.x(), body_walls->normal.y(), body_walls->normal.z()}},
                         {"body_center", {body_walls->center.x(), body_walls->center.y(), body_walls->center.z()}},
                         {"angle_offset_covariance", {{body_walls->covariance(0, 0), body_walls->covariance(0, 1)}, {body_walls->covariance(1, 0), body_walls->covariance(1, 1)}}}})
                        .dump(2);
          dedicated_geometry_cloud.reset();
        }
        for (const auto& point : points) leveled.push_back(q * point.head<3>());
        try {
          if (!body_walls) initial_walls = extract_local_walls(leveled);
          logger->info(
            "initial walls width={} center_offset={} rms={}; installation axis={},{} offset={}",
            initial_walls->width,
            initial_walls->center_offset,
            initial_walls->residual_rms,
            lateral_reference ? lateral_reference->normal.x() : 0.,
            lateral_reference ? lateral_reference->normal.y() : 0.,
            lateral_reference ? lateral_reference->offset : 0.);
        } catch (const std::invalid_argument& error) {
          logger->warn("initial wall extraction unavailable: {}", error.what());
        }
      }
      auto guess = fuse && !self_calibrate ? vector3(cfg.at("uwb").at("initial_position_guess")) : gtsam::Vector3::Zero();
      state = gtsam::NavState(gtsam::Pose3(gtsam::Rot3(q), guess), gtsam::Vector3::Zero());
      bias = gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), sum_w / initial_samples);
      if (fuse && !self_calibrate) {
        factors.emplace_shared<GravityAlignmentFactor>(X(0), avg, .001);
        if (variable_anchors()) {
          gtsam::KeyVector keys;
          for (size_t j = 0; j < map.ids.size(); j++) {
            keys.push_back(A(map.ids[j]));
            initial.insert(keys.back(), map.means[j]);
          }
          if (reuse_map) {
            initial.insert(gtsam::Symbol('l',0), calibrated_map->reference);
            // The entire first-run posterior enters once. No old ranges,
            // installation observation, vehicle state or bias are reintroduced.
            factors.emplace_shared<CalibratedMapPrior>(*calibrated_map);
          } else factors.emplace_shared<SurveyedMapPrior>(keys, map.means, map.covariance);
        }
      } else
        factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), state.pose(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
      if (body_walls && reuse_map) {
        factors.emplace_shared<WallReferenceFactor>(X(0), gtsam::Symbol('l',0), body_walls->normal, body_walls->center,
          body_walls->covariance, gtsam::Point3::Zero(), 1000.);
      }
      if (body_walls && !self_calibrate && !reuse_map) {
        const auto line_key = gtsam::Symbol('l', 0);
        // Center the line coordinates to avoid angle/offset scale disparity in
        // elimination. This is an exact reparameterization, not new information.
        gtsam::Point3 axis_origin = gtsam::Point3::Zero();
        for (const auto& point : map.means) axis_origin += point;
        axis_origin /= map.means.size();
        initial.insert(
          line_key,
          gtsam::Vector2(
            line_angle_scale * std::atan2(lateral_reference->normal.y(), lateral_reference->normal.x()),
            lateral_reference->offset - lateral_reference->normal.dot(axis_origin.head<2>())));
        const auto& records = map.installation.at("records");
        gtsam::KeyVector anchor_keys;
        gtsam::Vector offsets(records.size());
        gtsam::Matrix covariance(records.size(), records.size());
        for (size_t i = 0; i < records.size(); ++i) {
          anchor_keys.push_back(A(records[i].at("anchor_id").get<uint32_t>()));
          offsets[i] = records[i].at("signed_offset_m");
          for (size_t j = 0; j < records.size(); ++j) covariance(i, j) = map.installation.at("joint_covariance_m2")[i][j].get<double>();
        }
        factors.emplace_shared<InstallationOffsetFactor>(line_key, anchor_keys, offsets, covariance, axis_origin, line_angle_scale);
        factors.emplace_shared<WallReferenceFactor>(X(0), line_key, body_walls->normal, body_walls->center, body_walls->covariance, axis_origin, line_angle_scale);
      }
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), state.velocity(), gtsam::noiseModel::Isotropic::Sigma(3, .01));
      gtsam::Vector6 sig;
      sig.head<3>().setConstant(cfg.at("sensors").at("initial_acc_bias_sigma"));
      sig.tail<3>().setConstant(cfg.at("sensors").at("initial_gyro_bias_sigma"));
      factors.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), bias, gtsam::noiseModel::Diagonal::Sigmas(sig));
      initialized = true;
    } else {
      failure_context = "IMU_INTEGRATION_FAILED";
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
      failure_context = "GRAPH_BUILD_FAILED";
      int window = cfg.at("odometry").at("connection_window");
      size_t matching_keyframes = 0;
      const size_t matching_keyframe_limit = cfg.at("odometry").at("max_keyframes");
      for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        const bool recent = int(count - it->id) <= window;
        if (!recent && (!it->keyframe || (bounded_bootstrap_work && matching_keyframes >= matching_keyframe_limit))) continue;
        if (it->keyframe) ++matching_keyframes;
        // Raw states remain available for delayed ranges. Retaining a state
        // does not authorize adding every historical keyframe as a new match.
        gtsam::NonlinearFactor::shared_ptr f;
        if (backend == "gpu") {
          auto sb = streams->get_stream_buffer();
          f = gtsam::make_shared<GaugeConsistentVGICPFactorGPU>(X(it->id), X(count), it->voxels, match_cloud, sb.first, sb.second, lidar_weight, translation_projector);
        } else {
          auto cpu = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(X(it->id), X(count), it->voxels, match_cloud);
          cpu->set_num_threads(cfg.at("odometry").at("threads"));
          f = cpu;
        }
        lidar.add(f);
        factors.add(f);
      }
    }
    if (ongoing_walls) {
      failure_context = "CONTINUOUS_WALL_OBSERVATION_FAILED";
      const auto observation = body_wall_observation(wall_points, state.pose().rotation(), cfg.at("sensors").at("sigma_lidar_range"));
      gtsam::Point3 origin = gtsam::Point3::Zero();
      if (!self_calibrate && !reuse_map) {
        for (const auto& point : map.means) origin += point;
        origin /= map.means.size();
      }
      factors.emplace_shared<WallReferenceFactor>(X(count), gtsam::Symbol('l', 0), observation.normal, observation.center, observation.covariance, origin, line_angle_scale);
      std::ofstream evidence(RunLogManager::instance().profiling_path("continuous_walls.jsonl"), std::ios::app);
      evidence.exceptions(std::ios::failbit | std::ios::badbit);
      evidence << nlohmann::json({{"session", session},
                                  {"stamp", t},
                                  {"reserved_points", wall_points.size()},
                                  {"vgicp_points", points.size()},
                                  {"body_normal", {observation.normal.x(), observation.normal.y(), observation.normal.z()}},
                                  {"body_center", {observation.center.x(), observation.center.y(), observation.center.z()}},
                                  {"covariance", {{observation.covariance(0, 0), observation.covariance(0, 1)}, {observation.covariance(1, 0), observation.covariance(1, 1)}}},
                                  {"used_by_vgicp", false}})
                    .dump()
               << '\n';
    }
    failure_context = "GRAPH_BUILD_FAILED";
    double pred_x = state.position().x();
    const gtsam::Vector3 predicted_velocity = state.velocity();
    const auto predicted_rotation = state.pose().rotation();
    initial.insert(X(count), state.pose());
    initial.insert(V(count), state.velocity());
    initial.insert(B(count), bias);
    gtsam::Values predicted = bootstrapped ? smoother->calculateEstimate() : bootstrap_values;
    predicted.insert(initial);
    if (self_calibrate && !calibration_prepared) retain_calibration_ranges(t);
    else if (fuse) append_ranges(t, factors, predicted);
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    gtsam::Vector6 lidar_gradient = gtsam::Vector6::Zero();
    double fraction = 0, cost = 0;
    const auto diagnostics_start = std::chrono::steady_clock::now();
    gtsam_points::NonlinearFactorSetGPU gpu_factors;
    if (backend == "gpu") {
      gpu_factors.add(lidar);
      gpu_factors.linearize(predicted);
      gpu_factors.error(predicted);
    }
    for (const auto& f : lidar) {
      gtsam::HessianFactor h(*f->linearize(predicted));
      H += h.hessianBlockDiagonal().at(X(count));
      lidar_gradient += h.gradientAtZero().at(X(count));
      cost += f->error(predicted);
      fraction += backend == "gpu" ? dynamic_cast<gtsam_points::IntegratedVGICPFactorGPU*>(f.get())->inlier_fraction()
                                   : dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(f.get())->inlier_fraction();
    }
    if (!lidar.empty()) fraction /= lidar.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(H);
    auto eigen = eig.eigenvalues();
    auto weak = eig.eigenvectors().col(0).eval();
    gtsam::FixedLagSmoother::KeyTimestampMap times{{X(count), t}, {V(count), t}, {B(count), t}};
    if (fuse && variable_anchors())
      for (auto id : map.ids) times[A(id)] = t;
    if (body_walls && (!self_calibrate || calibration_prepared)) times[gtsam::Symbol('l', 0)] = t;
    failure_context = "OPTIMIZATION_FAILED";
    const auto solver_start = std::chrono::steady_clock::now();
    if (!bootstrapped) {
      bootstrap_factors.add(factors);
      bootstrap_values.insert(initial);
      for (const auto& kv : times) bootstrap_times[kv.first] = kv.second;
      if (self_calibrate && !calibration_prepared && t - first_imu >= cfg.at("uwb").at("bootstrap_s").get<double>()) prepare_calibration(t);
      if (self_calibrate && !calibration_prepared) {
        // Retain the original graph, not a trajectory pseudo-measurement. No
        // smoother marginalization occurs before unknown anchors are resolved.
        if (t - first_imu >= 60. || count >= 600) {
          failure_context = "SELF_CALIBRATION_PENDING_TIMEOUT";
          throw std::runtime_error("unknown-anchor joint initialization not certified within bounded startup");
        }
        frames.push_back({count, t, voxel, count % 10 == 0});
        last_cloud = t;
        ++count;
        status("INITIALIZING", "UNKNOWN_ANCHORS_PENDING_JOINT_INITIALIZATION", t);
        return;
      }
      const double bootstrap_duration = cfg.at("uwb").at("bootstrap_s");
      if (t - first_imu < bootstrap_duration) {
        frames.push_back({count, t, voxel, count % 10 == 0});
        last_cloud = t;
        count++;
        status("INITIALIZING", "MAP_FRAME_JOINT_BOOTSTRAP", t);
        return;
      }
      // Before the first solve, each static pair enters once with the exact
      // accumulated raw likelihood, including its state-independent cost.
      aa_added.clear();
      if (!cfg.at("uwb").value("bootstrap_multistart", false))
        for (const auto& entry : aa_summaries) {
          aa_added.emplace_back(entry.first, bootstrap_factors.size());
          const auto& stats = entry.second.statistics;
          bootstrap_factors.emplace_shared<AnchorAnchorRangeFactor>(A(entry.first.first), A(entry.first.second), stats.mean, stats.sigma(), stats.scatter);
        }
      gtsam_points::LevenbergMarquardtExtParams params;
      params.maxIterations = cfg.at("uwb").value("bootstrap_max_iterations", 30);
      params.relativeErrorTol = cfg.at("uwb").value("bootstrap_relative_error_tolerance", 1e-5);
      auto write_bootstrap_state = [this](const gtsam::Values& values, const char* event) {
        double ta_error = 0;
        gtsam::Vector3 gradient = gtsam::Vector3::Zero();
        for (const auto& factor : bootstrap_factors) {
          const auto* range = dynamic_cast<const TagAnchorRangeFactor*>(factor.get());
          if (!range) continue;
          std::vector<gtsam::Matrix> jacobians;
          auto residual = range->unwhitenedError(values, &jacobians);
          gtsam::Matrix world_H = gtsam::Matrix::Zero(1, 3);
          for (size_t j = 0; j < range->keys().size(); j++) {
            auto key = range->keys()[j];
            if (gtsam::Symbol(key).chr() == 'x') world_H += jacobians[j].rightCols(3) * values.at<gtsam::Pose3>(key).rotation().matrix().transpose();
          }
          gradient += range->noiseModel()->Whiten(world_H).transpose() * range->noiseModel()->whiten(residual);
          ta_error += range->error(values);
        }
        auto p = values.at<gtsam::Pose3>(X(0)).translation();
        bootstrap_log << ta_error << ',' << gradient.norm() << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << event << '\n';
      };
      params.callback = [&](const auto& progress, const gtsam::Values& values) {
        bootstrap_log << session << ',' << progress.iterations << ',' << progress.error << ',' << progress.cost_change << ',' << progress.lambda << ',' << progress.solve_success
                      << ',' << progress.elapsed_time << ',';
        write_bootstrap_state(values, "iteration");
      };
      params.status_msg_callback = [this](const std::string& message) { logger->info("bootstrap optimizer: {}", message); };
      gtsam::Values optimized;
      if (cfg.at("uwb").value("bootstrap_multistart", false)) {
        // Branch results are logged separately; do not interleave the legacy
        // single-optimizer callback with incomparable candidate iterations.
        auto branch_params = params;
        branch_params.callback = nullptr;
        auto result = solve_bootstrap_branches(t, branch_params);
        if (!result) {
          frames.push_back({count, t, voxel, count % 10 == 0});
          last_cloud = t;
          ++count;
          return;
        }
        optimized = *result;
      } else {
        gtsam_points::LevenbergMarquardtOptimizerExt optimizer(bootstrap_factors, bootstrap_values, params);
        optimized = optimizer.optimize();
        bootstrap_log << session << ',' << optimizer.iterations() << ',' << optimizer.error() << ",," << optimizer.lambda() << ",,,";
        write_bootstrap_state(optimized, "returned");
      }
      if (cfg.at("uwb").value("bootstrap_translation_probe", false)) {
        if (backend != "gpu") throw std::runtime_error("bootstrap translation probe requires GPU");
        gtsam::Matrix3 H = gtsam::Matrix3::Zero();
        gtsam::Vector3 g = gtsam::Vector3::Zero();
        gtsam::NonlinearFactorGraph probe;
        for (const auto& factor : bootstrap_factors) {
          if (dynamic_cast<const GaugeConsistentVGICPFactorGPU*>(factor.get()))
            probe.push_back(factor->clone());
          else
            probe.push_back(factor);
          const auto* range = dynamic_cast<const TagAnchorRangeFactor*>(factor.get());
          if (!range) continue;
          std::vector<gtsam::Matrix> J;
          auto e = range->unwhitenedError(optimized, &J);
          gtsam::Matrix world = gtsam::Matrix::Zero(1, 3);
          for (size_t j = 0; j < range->keys().size(); ++j) {
            const auto key = range->keys()[j];
            if (gtsam::Symbol(key).chr() == 'x') world += J[j].rightCols(3) * optimized.at<gtsam::Pose3>(key).rotation().matrix().transpose();
          }
          const auto A = range->noiseModel()->Whiten(world);
          H += A.transpose() * A;
          g += A.transpose() * range->noiseModel()->whiten(e);
        }
        Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eig(H);
        if (eig.info() != Eigen::Success || eig.eigenvalues()[0] <= 1e-12) {
          logger->warn("bootstrap translation probe invalid: singular curvature");
        } else {
          const gtsam::Vector3 step = -H.ldlt().solve(g);
          if (!step.allFinite()) throw std::runtime_error("nonfinite bootstrap translation probe step");
          gtsam_points::LinearizationHook hook(probe);
          hook.linearize(optimized);
          for (const auto& factor : probe)
            if (dynamic_cast<const GaugeConsistentVGICPFactorGPU*>(factor.get())) factor->linearize(optimized);
          std::ofstream output(RunLogManager::instance().profiling_path("bootstrap_translation_probe.csv"), std::ios::app);
          output << std::setprecision(16);
          if (output.tellp() == 0) output << "session,scale,dx,dy,dz,ta_cost,other_cost,eig_min,eig_mid,eig_max,gx,gy,gz\n";
          for (double scale : {0., 1., .5, .25, .125, .0625, .03125, .015625}) {
            auto trial = optimized;
            for (const auto key : optimized.keys()) {
              if (gtsam::Symbol(key).chr() != 'x') continue;
              const auto pose = optimized.at<gtsam::Pose3>(key);
              trial.update(key, gtsam::Pose3(pose.rotation(), pose.translation() + scale * step));
            }
            hook.error(trial);
            double ta = 0, other = 0;
            for (const auto& factor : probe) {
              const double cost = factor->error(trial);
              if (dynamic_cast<const TagAnchorRangeFactor*>(factor.get()))
                ta += cost;
              else
                other += cost;
            }
            output << session << ',' << scale << ',' << scale * step.x() << ',' << scale * step.y() << ',' << scale * step.z() << ',' << ta << ',' << other << ','
                   << eig.eigenvalues()[0] << ',' << eig.eigenvalues()[1] << ',' << eig.eigenvalues()[2] << ',' << g.x() << ',' << g.y() << ',' << g.z() << '\n';
          }
        }
      }
      // Transfer the original factors and their joint optimized values exactly
      // once. No marginal-map + reused-observation double counting.
      if (self_calibrate) {
        auto compact = marginalize_joint(bootstrap_factors, optimized, calibration_elimination(optimized, t, false));
        bootstrap_factors = std::move(compact.graph);
        optimized = std::move(compact.values);
        for (auto it = bootstrap_times.begin(); it != bootstrap_times.end();) {
          if (!optimized.exists(it->first)) it = bootstrap_times.erase(it); else ++it;
        }
        // Compaction changes factor indices. Static AA summaries were not
        // absorbed; bind their actual replacement indices, never add them twice.
        aa_added.clear();
        for (size_t index = 0; index < bootstrap_factors.size(); ++index) {
          const auto* aa = dynamic_cast<const AnchorAnchorRangeFactor*>(bootstrap_factors[index].get());
          if (!aa) continue;
          auto a = uint32_t(gtsam::Symbol(aa->keys()[0]).index()), b = uint32_t(gtsam::Symbol(aa->keys()[1]).index());
          aa_added.emplace_back(std::make_pair(std::min(a, b), std::max(a, b)), index);
        }
      }
      smoother->update(bootstrap_factors, optimized, bootstrap_times);
      bind_aa_factor_indices();
      bootstrap_factors.resize(0);
      bootstrap_values.clear();
      bootstrap_times.clear();
      bootstrapped = true;
      bootstrap_branches.clear();
      stationary_seed_ranges.clear();
    } else {
      smoother->update(factors, initial, times, aa_removed);
      bind_aa_factor_indices();
    }
    if (self_calibrate) advance_anchor_extension(t, factors);
    // Bounded diagnostic: re-optimize the SAME early graph without new data.
    // Never enabled implicitly or extended after a failed experiment.
    if (fuse && cfg.at("odometry").value("early_five_extra_updates", false) && t <= 13.0) {
      for (int iteration = 0; iteration < 5; ++iteration) smoother->update();
    }
    // Opt-in common-translation invariant probe before vehicle marginalization.
    // Relative GPU factors are not relinearized; their caches remain untouched.
    const bool yaw_probe = cfg.at("odometry").value("yaw_information_probe", false);
    if (yaw_probe && backend != "gpu") throw std::runtime_error("yaw information probe requires GPU");
    if (fuse && (yaw_probe || cfg.at("odometry").value("translation_information_probe", false)) && t <= 13.0) {
      const auto& theta = smoother->getLinearizationPoint();
      gtsam::GaussianFactorGraph ta;
      for (const auto& factor : smoother->getFactors()) {
        if (factor && (yaw_probe ? !dynamic_cast<const gtsam_points::IntegratedVGICPFactorGPU*>(factor.get()) : dynamic_cast<const TagAnchorRangeFactor*>(factor.get()) != nullptr))
          ta.push_back(factor->linearize(theta));
      }
      const auto& cached = smoother->getLinearFactors();
      gtsam::GaussianFactorGraph tree(smoother->getISAM2());
      const auto zero = theta.zeroVectors();
      const auto path = RunLogManager::instance().profiling_path(yaw_probe ? "yaw_information_probe.csv" : "translation_information_probe.csv");
      const bool header = !std::filesystem::exists(path);
      std::ofstream output(path, std::ios::app);
      output.exceptions(std::ios::failbit | std::ios::badbit);
      if (header)
        output
          << (yaw_probe ? "session,stamp,axis,raw_gradient,cached_gradient,tree_gradient,raw_curvature,cached_curvature,tree_curvature\n"
                        : "session,stamp,axis,ta_gradient,cached_gradient,tree_gradient,ta_curvature,cached_curvature,tree_curvature\n");
      output << std::setprecision(16);
      for (int axis = yaw_probe ? 3 : 0; axis < (yaw_probe ? 4 : 3); ++axis) {
        auto positive = zero, negative = zero;
        for (const auto key : theta.keys()) {
          if (yaw_probe && gtsam::Symbol(key).chr() == 'v') {
            const gtsam::Vector3 direction = gtsam::Vector3::UnitZ().cross(theta.at<gtsam::Vector3>(key));
            positive[key] = direction;
            negative[key] = -direction;
          }
          if (gtsam::Symbol(key).chr() != 'x') continue;
          gtsam::Vector6 direction = gtsam::Vector6::Zero();
          const auto& pose = theta.at<gtsam::Pose3>(key);
          if (yaw_probe) {
            direction.head<3>() = pose.rotation().matrix().transpose() * gtsam::Vector3::UnitZ();
            direction.tail<3>() = pose.rotation().matrix().transpose() * gtsam::Vector3::UnitZ().cross(pose.translation());
          } else {
            direction.tail<3>() = pose.rotation().matrix().transpose().col(axis);
          }
          positive[key] = direction;
          negative[key] = -direction;
        }
        auto moments = [&](const gtsam::GaussianFactorGraph& graph) {
          const double ep = graph.error(positive), em = graph.error(negative), e0 = graph.error(zero);
          return std::pair<double, double>{0.5 * (ep - em), ep + em - 2.0 * e0};
        };
        const auto a = moments(ta), b = moments(cached), c = moments(tree);
        output << session << ',' << t << ',' << axis << ',' << a.first << ',' << b.first << ',' << c.first << ',' << a.second << ',' << b.second << ',' << c.second << '\n';
      }
    }
    // Read-only post-fit consistency of newly incorporated physical TA factors.
    // Evaluate against the joint optimized states, not published poses or truth.
    uwb_geometry_trace = std::numeric_limits<double>::infinity();
    if (fuse && new_ta) {
      const auto estimate = smoother->calculateEstimate();
      size_t samples = 0;
      double squared_sum = 0;
      gtsam::Matrix3 geometry_information = gtsam::Matrix3::Zero();
      for (const auto& factor : factors) {
        const auto* range = dynamic_cast<const TagAnchorRangeFactor*>(factor.get());
        if (!range) continue;
        squared_sum += 2.0 * range->error(estimate);
        geometry_information += range->optimistic_translation_information(estimate);
        ++samples;
      }
      if (samples != new_ta) throw std::runtime_error("POSTFIT_TA_COUNT_MISMATCH");
      uwb_geometry_trace = optimistic_position_trace(geometry_information);
      postfit_ta_count = samples;
      postfit_ta_squared_sum = squared_sum;
      postfit_ta_stamp = t;
      failed_recovery_stamp = recovery_failure_stamp(failed_recovery_stamp, t, uwb_recovery_until, samples, squared_sum);
    }
    state = gtsam::NavState(smoother->calculateEstimate<gtsam::Pose3>(X(count)), smoother->calculateEstimate<gtsam::Vector3>(V(count)));
    bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(count));
    if (self_calibrate && (count % 10 == 0 || map_export_session != session || map_export_anchor_count != map.ids.size())) export_calibrated_map(t);
    if (!state.pose().matrix().allFinite() || !state.velocity().allFinite()) throw std::runtime_error("nonfinite optimized state");
    failure_context = "COVARIANCE_FAILED";
    const auto covariance_start = std::chrono::steady_clock::now();
    gtsam::Matrix xv_cov;
    gtsam::Vector6 twist = gtsam::Vector6::Zero();
    gtsam::Matrix66 twist_cov = gtsam::Matrix66::Zero();
    if (count > 0) {
      const auto joint = selected_covariance(smoother->getISAM2(), {X(count - 1), X(count), V(count)});
      xv_cov = joint.bottomRightCorner(9, 9);
      Eigen::Matrix<double, 6, 15> H;
      twist = interval_body_twist(smoother->calculateEstimate<gtsam::Pose3>(X(count - 1)), state.pose(), state.velocity(), t - last_cloud, &H);
      twist_cov = H * joint * H.transpose();
      if (!twist.allFinite() || !twist_cov.allFinite() || twist_cov.diagonal().minCoeff() <= 0) throw std::runtime_error("invalid joint body twist covariance");
    } else {
      xv_cov = joint_covariance(smoother->getISAM2(), X(count), V(count));
    }
    gtsam::Matrix66 cov = xv_cov.topLeftCorner<6, 6>();
    if (!cov.allFinite() || cov.diagonal().minCoeff() < 0) throw std::runtime_error("invalid pose covariance");
    const auto covariance_end = std::chrono::steady_clock::now();
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
      if (num_keys <= cfg.at("odometry").value("max_keyframes", 15u)) break;
      if (f.keyframe) {
        f.keyframe = false;
        num_keys--;
      }
    }
    if (count % 10 == 0) {
      std::ifstream statm("/proc/self/statm");
      size_t virtual_pages = 0, resident_pages = 0;
      if (!(statm >> virtual_pages >> resident_pages)) throw std::runtime_error("RESOURCE_RSS_UNAVAILABLE");
      size_t device_free = 0, device_total = 0;
      if (backend == "gpu" && cudaMemGetInfo(&device_free, &device_total) != cudaSuccess) throw std::runtime_error("RESOURCE_DEVICE_MEMORY_UNAVAILABLE");
      resources << session << ',' << t << ',' << resident_pages * sysconf(_SC_PAGESIZE) / 1024 << ',' << device_total - device_free << ',' << frames.size() << ',' << num_keys
                << ',' << imu->imu_data_in_queue().size() << ',' << clouds.size() << ',' << range_queue.size() << '\n';
    }
    auto finish = std::chrono::steady_clock::now();
    auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    std::string mode = lidar.empty() ? "INITIALIZING" : (fraction < .5 || eigen[0] < 1e-6 || eigen[5] / std::max(eigen[0], 1e-12) > 1e6 ? "DEGRADED" : "TRACKING");
    std::string state_reason = "LIDAR_INFORMATION_DIAGNOSTIC_NO_DOWNWEIGHTING";
    if (fuse) {
      const auto diagnostic = fused_status(
        t,
        last_uwb,
        last_rejected,
        last_rejection,
        cov.bottomRightCorner<3, 3>().trace(),
        cov.trace(),
        uwb_recovery_until,
        postfit_ta_count,
        postfit_ta_squared_sum,
        failed_recovery_stamp,
        uwb_geometry_trace);
      mode = diagnostic.state;
      state_reason = diagnostic.reason;
      if ((self_calibrate || reuse_map) && mode == "TRACKING") {
        mode = "DEGRADED";
        state_reason = "CALIBRATED_MAP_VALIDITY_UNVERIFIED";
      }
    }
    failure_context = "STATE_OUTPUT_FAILED";
    if (count > 0) {
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
      out.twist.twist.linear.x = twist[0];
      out.twist.twist.linear.y = twist[1];
      out.twist.twist.linear.z = twist[2];
      out.twist.twist.angular.x = twist[3];
      out.twist.twist.angular.y = twist[4];
      out.twist.twist.angular.z = twist[5];
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) out.twist.covariance[i * 6 + j] = twist_cov(i, j);
      pub->publish(out);
      auto publish_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
      status(mode, state_reason, t);
      trajectory << session << ',' << t << ',' << now().seconds() << ',' << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
                 << ',' << v.x() << ',' << v.y() << ',' << v.z() << ",1," << mode << "," << (fuse ? map.id : "none") << "," << publish_wall_ns << "," << state_reason << ","
                 << twist[3] << "," << twist[4] << "," << twist[5] << "," << t - last_cloud << "," << postfit_ta_stamp << "," << postfit_ta_count << "," << postfit_ta_squared_sum
                 << "," << postfit_range_limit(postfit_ta_count) << "," << failed_recovery_stamp << "," << uwb_geometry_trace << "\n";
      covariance_log << session << ',' << t;
      for (int i = 0; i < 9; i++)
        for (int j = 0; j < 9; j++) covariance_log << ',' << xv_cov(i, j);
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++) covariance_log << ',' << twist_cov(i, j);
      covariance_log << '\n';
    } else {
      status("INITIALIZING", "WAIT_SECOND_POSE_FOR_TWIST", t);
    }
    size_t anchors = fuse && variable_anchors() ? map.ids.size() : 0;
    size_t joint_map_priors = 0, surveyed_priors = 0, installation_observations = 0;
    for (const auto& factor : smoother->getFactors()) {
      if (!factor) continue;
      joint_map_priors += dynamic_cast<const CalibratedMapPrior*>(factor.get()) != nullptr;
      surveyed_priors += dynamic_cast<const SurveyedMapPrior*>(factor.get()) != nullptr;
      installation_observations += dynamic_cast<const InstallationOffsetFactor*>(factor.get()) != nullptr;
    }
    if (self_calibrate && installation_observations != 1) throw std::runtime_error("CALIBRATION_INSTALLATION_INFORMATION_LIFECYCLE_VIOLATION");
    if (reuse_map && (joint_map_priors != 1 || surveyed_priors != 0 || installation_observations != 0))
      throw std::runtime_error("REUSE_MAP_INFORMATION_LIFECYCLE_VIOLATION");
    size_t active = (smoother->timestamps().size() - anchors) / 3;
    graph << session << ',' << t << ',' << active << ',' << active << ',' << active << ',' << anchors << ',' << (count ? 1 : 0) << ',' << lidar.size() << ','
          << smoother->getFactors().nrFactors() << ',' << retired << ',' << ms(preprocessed, finish) << ',' << (session - 1) << ",0," << new_ta << ',' << new_aa << ','
          << (joint_map_priors + surveyed_priors) << ',' << range_queue.size() << ',' << aa_summaries.size() << ',' << aa_measurements
          << ',' << joint_map_priors << ',' << installation_observations << "\n";
    icp << session << ',' << t << ',' << points.size() << ',' << fraction << ',' << cost;
    for (int i = 0; i < 6; i++) icp << ',' << eigen[i];
    for (int i = 0; i < 6; i++) icp << ',' << weak[i];
    icp << ",covariance_planarity_guard_tangent_variance=" << tangent_variance << "_native_cost_weight=" << lidar_weight << "," << nonplanar << ',' << surface_spectrum[0] << ','
        << surface_spectrum[1] << ',' << surface_spectrum[2] << ',' << translation_projector.trace();
    for (int i = 0; i < 6; ++i) icp << ',' << lidar_gradient[i];
    const gtsam::Vector3 world_gradient = predicted_rotation.rotate(lidar_gradient.tail<3>());
    for (int i = 0; i < 3; ++i) icp << ',' << world_gradient[i];
    icp << '\n';
    timing << session << ',' << t << ',' << ms(start, preprocessed) << ',' << ms(preprocessed, finish) << ',' << ms(start, finish) << ',' << now().seconds() - t << ','
           << imu->imu_data_in_queue().size() << ',' << clouds.size() << ',' << pred_x;
    for (int i = 0; i < 6; i++) timing << ',' << bias.vector()[i];
    timing << ',' << ms(diagnostics_start, solver_start) << ',' << ms(solver_start, covariance_start) << ',' << ms(covariance_start, covariance_end);
    for (int i = 0; i < 3; ++i) timing << ',' << predicted_velocity[i];
    timing << '\n';
    last_cloud = t;
    count++;
  }
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> streams;
  std::string backend, gpu_name;
  bool fuse = false, bootstrapped = true, self_calibrate = false, calibration_prepared = false, reuse_map = false, bounded_bootstrap_work = false;
  std::optional<CalibratedMap> calibrated_map;
  size_t map_export_session = 0, map_export_anchor_count = 0;
  std::optional<gtsam::Values> calibration_previous;
  std::set<size_t> calibration_solved;
  std::optional<InstallationInput> installation_input;
  struct PendingCalibrationRange { RangeObservation observation; RangeBracket bracket; };
  std::vector<PendingCalibrationRange> pending_calibration_ranges;
  std::vector<PendingCalibrationRange> extension_ranges;
  struct ExtensionBatch {
    uint32_t id = 0;
    double stamp = 0, oldest = 0;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<gtsam::Point3> seeds, solutions;
    std::vector<double> costs;
    std::unique_ptr<SteppedBootstrapLM> optimizer;
    std::exception_ptr failure;
  };
  std::unique_ptr<ExtensionBatch> extension_batch;
  std::map<uint32_t,gtsam::Point3> extension_previous;
  std::map<uint32_t,double> extension_last_attempt;
  KnownMap map;
  std::optional<LateralReference> lateral_reference;
  std::optional<LocalWallPair> initial_walls;
  std::optional<BodyWallObservation> body_walls;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr dedicated_geometry_cloud;
  gtsam::Point3 tag;
  RangeQueue range_queue;
  double last_uwb = -1, last_rejected = -1, uwb_recovery_until = -1;
  double uwb_geometry_trace = std::numeric_limits<double>::infinity();
  double postfit_ta_stamp = -1, postfit_ta_squared_sum = 0, failed_recovery_stamp = -1;
  size_t postfit_ta_count = 0;
  uint64_t rejected_total = 0;
  std::string last_rejection, failure_context = "STARTUP_FAILED";
  size_t new_ta = 0, new_aa = 0;
  struct AnchorPairSummary {
    GaussianRangeSummary statistics;
    std::optional<gtsam::FactorIndex> factor_index;
  };
  std::map<std::pair<uint32_t, uint32_t>, AnchorPairSummary> aa_summaries;
  std::vector<std::pair<std::pair<uint32_t, uint32_t>, size_t>> aa_added;
  gtsam::FactorIndices aa_removed;
  uint64_t aa_measurements = 0;
  gtsam::NonlinearFactorGraph bootstrap_factors;
  gtsam::Values bootstrap_values;
  std::map<uint32_t, GaussianRangeSummary> stationary_seed_ranges;
  std::vector<gtsam::Values> bootstrap_branches;
  struct BootstrapBatch {
    double stamp = 0;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial, linearization_values;
    std::vector<BootstrapBranchEvidence> evidence;
    std::unique_ptr<SteppedBootstrapLM> optimizer;
  };
  std::unique_ptr<BootstrapBatch> bootstrap_batch;
  std::exception_ptr bootstrap_idle_failure;
  double bootstrap_last_attempt = -1;
  std::optional<gtsam::Pose3> bootstrap_previous_winner;
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
  std::ofstream trajectory, graph, icp, uwb, timing, covariance_log, bootstrap_log, resources, status_log;
  std::shared_ptr<spdlog::logger> logger;
  std::string last_status, last_status_reason;
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
  std::shared_ptr<liuo_tunnel::TunnelNode> node;
  try {
    liuo_tunnel::RunLogManager::initialize("odometry");
    node = std::make_shared<liuo_tunnel::TunnelNode>();
    rclcpp::spin(node);
    return 0;
  } catch (const std::exception& e) {
    spdlog::error("odometry failed: {}", e.what());
    try {
      if (node)
        node->report_failure(e.what());
      else if (auto* logs = liuo_tunnel::RunLogManager::get_if_initialized()) {
        std::ofstream out(logs->runtime_path("odometry_failure.json"));
        const std::string detail = e.what();
        const std::string reason = detail.rfind("MAP_INVALID:", 0) == 0 ? "MAP_INVALID" : (detail.rfind("EXTRINSIC_INVALID:", 0) == 0 ? "EXTRINSIC_INVALID" : "STARTUP_FAILED");
        out << nlohmann::json({{"state", "LOST"}, {"reason", reason}, {"detail", detail}}).dump(2) << '\n';
      }
    } catch (const std::exception& reporting) {
      spdlog::error("failure evidence could not be written: {}", reporting.what());
    }
    rclcpp::shutdown();
    return 1;
  }
}
