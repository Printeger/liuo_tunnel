#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <liuo_uwb_msgs/msg/uwb_observation_batch.hpp>
#include <liuo_tunnel/common/sensors.hpp>
#include <liuo_tunnel/util/run_log_manager.hpp>
#include <liuo_tunnel/util/logging.hpp>
#include <liuo_tunnel_sim/rail_model.hpp>
#include <liuo_tunnel_sim/rail_world.hpp>
#include <liuo_tunnel_sim/first_hit_lidar_renderer.hpp>
#include <fstream>
#include <map>
#include <limits>
#include <iomanip>
#include <chrono>
#include <sys/resource.h>
using liuo_tunnel::RunLogManager;
namespace liuo_tunnel_sim {
class RailSim : public rclcpp::Node {
public:
  RailSim() : Node("liuo_rail_sim"), renderer(make_renderer()) {
    cfg = liuo_tunnel::load_config(declare_parameter<std::string>("config_file", ""));
    for (const auto& node : cfg.at("uwb").at("input").at("nodes")) {
      if (node.at("type") == 1) logical_anchors.emplace(node.at("internal_id").get<int>(), node.at("logical_id").get<uint8_t>());
      else logical_tag = node.at("logical_id").get<uint8_t>();
    }
    required_sensor_consumers = declare_parameter<int>("required_sensor_consumers", 2);
    if (required_sensor_consumers < 1 || required_sensor_consumers > 2) throw std::runtime_error("required sensor consumers must be 1 or 2");
    seed = declare_parameter<int>("seed", 0);
    duration = declare_parameter<double>("duration_s", 45);
    rate = declare_parameter<double>("rate", 1.0);
    if (!std::isfinite(rate) || rate <= 0 || rate > 1) throw std::runtime_error("rate must be in (0,1]; explicit reference playback only");
    reset_at = declare_parameter<double>("reset_at_s", -1);
    return_trip = declare_parameter<bool>("return_trip", false);
    local_calibration = declare_parameter<bool>("local_calibration", false);
    calibration_route = declare_parameter<bool>("calibration_route", false);
    if (calibration_route && !local_calibration) throw std::runtime_error("calibration route requires local bootstrap maneuver");
    if (local_calibration && return_trip) throw std::runtime_error("incompatible simulation trajectories");
    start = {declare_parameter<double>("start_x", 0), declare_parameter<double>("start_y", 0), declare_parameter<double>("start_z", 1.2)};
    start_yaw = declare_parameter<double>("start_yaw", 0);
    if (!start.allFinite() || !std::isfinite(start_yaw)) throw std::runtime_error("nonfinite simulation start");
    T_BL = liuo_tunnel::lidar_extrinsic(cfg);
    tag = liuo_tunnel::vector3(cfg.at("sensors").at("T_BU_translation"));
    session = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    logger = liuo_tunnel::create_module_logger("simulation");
    auto& logs = RunLogManager::instance();
    logs.write_run_info(
      {{"scan_model", "snapshot"},
       {"renderer_threads", "4"},
       {"pacing", "absolute_deadline_no_drop"},
       {"seed", std::to_string(seed)},
       {"motion", return_trip ? "outbound_and_reverse_return" : "outbound"},
       {"producer_session", std::to_string(session)}});
    if (declare_parameter<bool>("capture", false)) {
      rosbag2_storage::StorageOptions options;
      options.uri = logs.export_path("capture/raw").string();
      options.storage_id = "mcap";
      options.storage_preset_profile = "zstd_fast";
      writer = std::make_unique<rosbag2_cpp::Writer>();
      writer->open(options);
    }
    wall_clock.open(logs.export_path("simulation/wall_clock.csv"));
    wall_clock << std::setprecision(16) << "session,stamp,scheduled_wall_ns,published_wall_ns\n";
    truth.open(logs.export_path("simulation/truth.csv"));
    truth << std::setprecision(16) << "session,stamp,x,y,z,vx,vy,vz,available_ta_count,available_ta_ids,qx,qy,qz,qw\n";
    uwblog.open(logs.export_path("simulation/uwb.csv"));
    uwblog << std::setprecision(16) << "session,sequence,stamp,a_type,a,b,true_range,range,variance\n";
    scheduling.open(logs.export_path("simulation/scheduling.csv"));
    scheduling << "session,stamp,ta_published,aa_published,unreachable,occluded,slots,overdue_links,max_lateness_s\n";
    timing.open(logs.profiling_path("simulation.csv"));
    timing << "stamp,render_ms,dda_visits,points,world_points\n";
    clock_pub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 2048);
    imu_pub = create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", rclcpp::SensorDataQoS().keep_last(2048));
    cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/lidar/points", rclcpp::SensorDataQoS().keep_last(32));
    truth_pub = create_publisher<nav_msgs::msg::Odometry>("/sim/truth/odom", 100);
    const int uwb_depth = declare_parameter<int>("uwb_qos_depth", 100);
    if (uwb_depth <= 0 || uwb_depth > 10000) throw std::invalid_argument("INVALID_UWB_QOS_DEPTH");
    uwb_pub = create_publisher<liuo_uwb_msgs::msg::UwbObservationBatch>(declare_parameter<std::string>("uwb_topic", "/liuo/uwb/observations"), rclcpp::QoS(rclcpp::KeepLast(uwb_depth)).best_effort().durability_volatile());
    for (int i = 0; i < anchor_count; i++) links.push_back({0, 0, i, 0, 0});
    for (int i = 0; i < anchor_count; i++)
      for (int j = i + 1; j < anchor_count; j++) links.push_back({1, i, j, 0, 0});
    timer = create_wall_timer(std::chrono::milliseconds(1), [this] { service(); });
    logger->info("snapshot simulation ready seed={} duration={}s", seed, duration);
  }

private:
  static FirstHitLidarConfig make_renderer() {
    FirstHitLidarConfig c;
    c.threads = 4;
    c.max_range_m = 40;
    c.vertical_min_deg = -25;
    c.vertical_max_deg = 25;
    return c;
  }
  builtin_interfaces::msg::Time stamp(double t) { return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9))); }
  void world(double x) {
    int block = static_cast<int>(std::floor(x / 20));
    if (block == loaded) return;
    loaded = block;
    // Bounded local block window. UWB uses the complete analytic convex world, never this LiDAR cache.
    auto p = rail_world(block);
    world_points = p.size();
    std::string why;
    if (!renderer.loadWorld(p, &why)) throw std::runtime_error("renderer world: " + why);
  }
  template <class MessageT>
  void capture(const MessageT& message, const std::string& topic) {
    // Monotonic bag playback schedule even if the message's ROS clock resets.
    // Header effective times and producer identities are stored unchanged.
    if (writer) writer->write(message, topic, stamp(1 + total_ticks * .005));
  }
  void service() {
    if (!ready) {
      tick();
      return;
    }
    // Absolute deadline pacing catches up ALL fixed steps after rendering;
    // ROS timers alone skip missed periods. No dropped simulated samples.
    for (int work = 0; work < 200 && rclcpp::ok(); work++) {
      double due = std::chrono::duration<double>(std::chrono::steady_clock::now() - simulation_started).count() * rate;
      double scheduled = total_ticks * .005;
      if (scheduled > due) break;
      if (due - scheduled > .25) throw std::runtime_error("SIMULATION_DEADLINE_MISSED: backlog >250ms");
      tick();
    }
  }
  void tick() {
    if (!ready) {
      if (imu_pub->get_subscription_count() < static_cast<size_t>(required_sensor_consumers) || cloud_pub->get_subscription_count() < static_cast<size_t>(required_sensor_consumers) || uwb_pub->get_subscription_count() < static_cast<size_t>(required_sensor_consumers)) {
        if (std::chrono::steady_clock::now() - startup > std::chrono::seconds(20)) throw std::runtime_error("sensor subscriber readiness timed out");
        return;
      }
      ready = true;
      simulation_started = std::chrono::steady_clock::now();
      logger->info("all raw sensor consumers discovered; start simulation clock");
    }
    double elapsed = total_ticks * .005;
    if (elapsed >= duration) {
      double wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - simulation_started).count();
      struct rusage usage {};
      getrusage(RUSAGE_SELF, &usage);
      nlohmann::json result =
        {{"simulation_s", elapsed}, {"wall_s", wall_seconds}, {"real_time_factor", elapsed / wall_seconds}, {"peak_rss_kib", usage.ru_maxrss}, {"scan_model", "snapshot"}};
      result["published_counts"] = {{"imu", total_ticks}, {"clock", total_ticks}, {"cloud", published_clouds}, {"uwb", ta_published + aa_published}};
      std::ofstream(RunLogManager::instance().export_path("simulation/runtime.json")) << result.dump(2) << std::endl;
      if (writer) writer->close();
      logger->info("simulation finished ticks={} producer_session={} real_time_factor={}", total_ticks, session, elapsed / wall_seconds);
      rclcpp::shutdown();
      return;
    }
    if (reset_at > 0 && !reset_done && elapsed >= reset_at) {
      reset_done = true;
      ticks = 0;
      session++;
      sequence = 0;
      exchange = 0;
      ba.setZero();
      bg.setZero();
      for (auto& l : links) {
        l.next = 0;
        l.samples = 0;
      }
      logger->warn("explicit simulation clock reset session={}", session);
    }
    double t = ticks * .005, ros_t = 1 + t;
    auto m = calibration_route ? calibration_route_motion(t,start,start_yaw) : (local_calibration ? calibration_motion(t, start, start_yaw) : initialized_motion(t, return_trip, start, start_yaw));
    rosgraph_msgs::msg::Clock clock;
    clock.clock = stamp(ros_t);
    capture(clock, "/clock");
    clock_pub->publish(clock);
    auto wall_ns = [](auto point) { return std::chrono::duration_cast<std::chrono::nanoseconds>(point.time_since_epoch()).count(); };
    wall_clock << (reset_done ? 2 : 1) << ',' << ros_t << ',' << wall_ns(simulation_started) + std::llround(elapsed / rate * 1e9) << ','
               << wall_ns(std::chrono::steady_clock::now()) << '\n';
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp(ros_t);
    imu.header.frame_id = "imu";
    imu.orientation_covariance[0] = -1;
    auto f = specific_force(m);
    auto w = m.omega;
    auto s = cfg.at("sensors");
    for (int j = 0; j < 3; j++) {
      ba[j] += s.at("sigma_ba_density").get<double>() * std::sqrt(.005) * normal(seed, 20, ticks, j);
      bg[j] += s.at("sigma_bg_density").get<double>() * std::sqrt(.005) * normal(seed, 21, ticks, j);
      f[j] += ba[j] + s.at("sigma_acc_sample").get<double>() * normal(seed, 10, ticks, j);
      w[j] += bg[j] + s.at("sigma_gyro_sample").get<double>() * normal(seed, 11, ticks, j);
      imu.linear_acceleration_covariance[j * 3 + j] = std::pow(s.at("sigma_acc_sample").get<double>(), 2);
      imu.angular_velocity_covariance[j * 3 + j] = std::pow(s.at("sigma_gyro_sample").get<double>(), 2);
    }
    imu.linear_acceleration.x = f.x();
    imu.linear_acceleration.y = f.y();
    imu.linear_acceleration.z = f.z();
    imu.angular_velocity.x = w.x();
    imu.angular_velocity.y = w.y();
    imu.angular_velocity.z = w.z();
    capture(imu, "/sensors/imu");
    imu_pub->publish(imu);
    while (exchange * .004 <= t + 1e-10) {
      double event = exchange++ * .004;
      schedule(event);
    }
    if (ticks % 20 == 0) {
      world(m.p.x());
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = m.R;
      pose.translation() = m.p;
      auto scan = renderer.render(pose * T_BL);
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = stamp(ros_t);
      cloud.header.frame_id = "lidar";
      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2Fields(
        5,
        "x",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "y",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "z",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "intensity",
        1,
        sensor_msgs::msg::PointField::FLOAT32,
        "time",
        1,
        sensor_msgs::msg::PointField::FLOAT32);
      mod.resize(scan.hits.size());
      sensor_msgs::PointCloud2Iterator<float> px(cloud, "x"), py(cloud, "y"), pz(cloud, "z"), pi(cloud, "intensity"), pt(cloud, "time");
      size_t index = 0;
      for (const auto& hit : scan.hits) {
        Eigen::Vector3d q = liuo_tunnel::point_in_lidar(pose, T_BL, Eigen::Vector3d(hit.x, hit.y, hit.z));
        q *= 1 + s.at("sigma_lidar_range").get<double>() * normal(seed, 30, ticks / 20, index++) / q.norm();
        *px = q.x();
        *py = q.y();
        *pz = q.z();
        *pi = 1;
        *pt = 0;
        ++px;
        ++py;
        ++pz;
        ++pi;
        ++pt;
      }
      capture(cloud, "/sensors/lidar/points");
      cloud_pub->publish(cloud);
      published_clouds++;
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp(ros_t);
      odom.header.frame_id = "world";
      odom.child_frame_id = "imu";
      odom.pose.pose.position.x = m.p.x();
      odom.pose.pose.position.y = m.p.y();
      odom.pose.pose.position.z = m.p.z();
      const Eigen::Quaterniond rotation(m.R);
      odom.pose.pose.orientation.x = rotation.x();
      odom.pose.pose.orientation.y = rotation.y();
      odom.pose.pose.orientation.z = rotation.z();
      odom.pose.pose.orientation.w = rotation.w();
      const Eigen::Vector3d body_velocity = m.R.transpose() * m.v;
      odom.twist.twist.linear.x = body_velocity.x();
      odom.twist.twist.linear.y = body_velocity.y();
      odom.twist.twist.linear.z = body_velocity.z();
      capture(odom, "/sim/truth/odom");
      truth_pub->publish(odom);
      // Evaluation-only visibility: true 3-D range and the same physical LOS gate.
      // Never published to or consumed by the estimator.
      const Eigen::Vector3d tag_w = liuo_tunnel::tag_in_world(m.p, m.R, tag);
      int available_ta = 0;
      std::string available_ids;
      for (int i = 0; i < anchor_count; ++i) {
        if (!reachable(tag_w, anchor(i)) || !line_of_sight(tag_w, anchor(i))) continue;
        if (available_ta++) available_ids += ";";
        available_ids += std::to_string(i);
      }
      truth << session << ',' << ros_t << ',' << m.p.x() << ',' << m.p.y() << ',' << m.p.z() << ',' << m.v.x() << ',' << m.v.y() << ',' << m.v.z() << ',' << available_ta << ','
            << available_ids << ',' << rotation.x() << ',' << rotation.y() << ',' << rotation.z() << ',' << rotation.w() << '\n';
      timing << ros_t << ',' << scan.stats.render_latency_ms << ',' << scan.stats.dda_voxel_visits << ',' << scan.hits.size() << ',' << world_points << '\n';
    }
    if (ticks % 200 == 0) {
      size_t overdue = 0;
      double late = 0;
      for (const auto& l : links)
        if (l.next < t) {
          overdue++;
          late = std::max(late, t - l.next);
        }
      scheduling << session << ',' << ros_t << ',' << ta_published << ',' << aa_published << ',' << unreachable << ',' << occluded << ',' << exchange << ',' << overdue << ','
                 << late << '\n';
    }
    ticks++;
    total_ticks++;
  }
  struct Link {
    int type, a, b;
    double next;
    uint64_t samples;
  };
  void schedule(double t) {
    auto m = calibration_route ? calibration_route_motion(t,start,start_yaw) : (local_calibration ? calibration_motion(t, start, start_yaw) : initialized_motion(t, return_trip, start, start_yaw));
    Eigen::Vector3d tag_w = liuo_tunnel::tag_in_world(m.p, m.R, tag);
    // 9 TA-priority slots, then 1 AA-priority slot; oldest deadline within class.
    // The other class can use an otherwise idle slot. Never exceed 250 exchanges/s.
    int best = -1;
    double deadline = 1e100;
    for (size_t i = 0; i < links.size(); i++) {
      auto& l = links[i];
      if (l.next > t + 1e-9) continue;
      Eigen::Vector3d a = l.type ? anchor(l.a) : tag_w;
      if (!reachable(a, anchor(l.b))) {
        unreachable++;
        l.next = t + (l.type ? 1 : .05);
        continue;
      }
      if (!line_of_sight(a, anchor(l.b))) {
        occluded++;
        l.next = t + (l.type ? 1 : .05);
        continue;
      }
      double rank = l.next + (bool(l.type) == prefer_anchor_exchange(exchange - 1) ? 0 : 10000);
      if (rank < deadline) {
        deadline = rank;
        best = i;
      }
    }
    if (best < 0) return;
    auto& l = links[best];
    Eigen::Vector3d a = l.type ? anchor(l.a) : tag_w, b = anchor(l.b);
    // Full-world LOS checked independently of the local LiDAR block cache.
    liuo_uwb_msgs::msg::UwbObservationBatch u;
    liuo_uwb_msgs::msg::UwbObservation o;
    // Explicit inverse of the configured logical registry; internal map IDs stay unchanged.
    u.header.stamp = stamp(1 + t);
    u.driver_session_id = session;
    u.batch_seq = sequence++;
    u.receiver_id = logical_tag;
    u.trigger_type = l.type;
    o.kind = l.type;
    o.node_a = l.type ? logical_anchors.at(l.a) : logical_tag;
    o.node_b = logical_anchors.at(l.b);
    o.reporter_id = o.node_a;
    o.source_session_id = session;
    o.source_frame_seq = static_cast<uint32_t>(u.batch_seq);
    o.source_frame_seq_valid = true;
    o.shard_count = 1;
    o.source_frame_stamp = stamp(1 + t);
    o.received_stamp = u.header.stamp;
    o.time_basis = o.TIME_FRAME_LOCAL_MAPPED;
    o.time_uncertainty_s = 0;
    o.source_hold_age_ms = 65535;
    o.fp_rssi = o.rx_rssi = std::numeric_limits<float>::quiet_NaN();
    o.pair_measurement_freshness_verified = true;
    const double physical_range = range(a, b, seed, 1000 + l.type * 10000 + l.a * 100 + l.b, l.samples++);
    o.distance_m = static_cast<float>(physical_range);
    u.observations.push_back(o);
    capture(u, uwb_pub->get_topic_name());
    uwb_pub->publish(u);
    uwblog << session << ',' << u.batch_seq << ',' << 1 + t << ',' << l.type << ',' << l.a << ',' << l.b << ',' << (a - b).norm() << ',' << o.distance_m << ",0.01\n";
    if (l.type)
      aa_published++;
    else
      ta_published++;
    l.next = t + (l.type ? 1 : .05);
  }
  std::map<int, uint8_t> logical_anchors;
  uint8_t logical_tag = 0;
  std::unique_ptr<rosbag2_cpp::Writer> writer;
  bool ready = false;
  std::chrono::steady_clock::time_point startup = std::chrono::steady_clock::now(), simulation_started;
  nlohmann::json cfg;
  int seed = 0, loaded = -100;
  int required_sensor_consumers = 2;
  double duration = 45, reset_at = -1, rate = 1, start_yaw = 0;
  Eigen::Vector3d start{0, 0, 1.2};
  bool reset_done = false, return_trip = false, local_calibration = false, calibration_route = false;
  uint64_t ticks = 0, total_ticks = 0, exchange = 0, session = 1, sequence = 0;
  size_t world_points = 0;
  Eigen::Vector3d ba = Eigen::Vector3d::Zero(), bg = Eigen::Vector3d::Zero(), tag;
  Eigen::Isometry3d T_BL;
  FirstHitLidarRenderer renderer;
  std::vector<Link> links;
  std::ofstream truth, uwblog, timing, scheduling, wall_clock;
  uint64_t published_clouds = 0;
  uint64_t ta_published = 0, aa_published = 0, unreachable = 0, occluded = 0;
  std::shared_ptr<spdlog::logger> logger;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  rclcpp::Publisher<liuo_uwb_msgs::msg::UwbObservationBatch>::SharedPtr uwb_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub;
  rclcpp::TimerBase::SharedPtr timer;
};
}  // namespace liuo_tunnel_sim
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    RunLogManager::initialize("simulation");
    rclcpp::spin(std::make_shared<liuo_tunnel_sim::RailSim>());
    return 0;
  } catch (const std::exception& e) {
    spdlog::error("simulation failed: {}", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
