// Independent subscriber only. No estimation, truth subscription or map access.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <liuo_uwb_msgs/msg/uwb_observation_batch.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <set>
#include <tuple>

int main(int argc, char** argv) {
  if (argc != 6) return 2;
  std::array<size_t, 4> expected{}, count{};
  for (int i = 0; i < 4; ++i) expected[i] = std::stoull(argv[i + 1]);
  const std::string report = argv[5];
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("road_scene_wire_checker");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(4096)).best_effort().durability_volatile();
  bool valid = true;
  std::set<std::tuple<uint64_t, uint8_t, uint64_t, uint32_t, uint8_t>> ids;
  auto cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>("/sensors/lidar", qos, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
    ++count[0];
    valid &= m->point_step == 24 && m->height == 1 && m->data.size() == m->width * 24 && m->fields.size() == 6;
    float previous = -1;
    for (size_t i = 0; i < m->width && valid; ++i) {
      uint16_t ring;
      float t, x, y, z;
      auto p = m->data.data() + 24 * i;
      std::memcpy(&x, p, 4);
      std::memcpy(&y, p + 4, 4);
      std::memcpy(&z, p + 8, 4);
      std::memcpy(&ring, p + 16, 2);
      std::memcpy(&t, p + 20, 4);
      valid &= std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && ring < 40 && t >= 0 && t < .1 && t > previous;
      valid &= static_cast<int>(std::llround(t * 204800)) % 40 == ring;
      previous = t;
    }
  });
  auto imu = node->create_subscription<sensor_msgs::msg::Imu>("/sensors/imu", qos, [&](sensor_msgs::msg::Imu::ConstSharedPtr m) {
    ++count[1];
    valid &= m->orientation_covariance[0] == -1 && std::isfinite(m->linear_acceleration.z) && std::isfinite(m->angular_velocity.z);
  });
  auto uwb = node->create_subscription<liuo_uwb_msgs::msg::UwbObservationBatch>("/liuo/uwb/observations", qos, [&](liuo_uwb_msgs::msg::UwbObservationBatch::ConstSharedPtr m) {
    ++count[2];
    for (const auto& o : m->observations) {
      valid &= o.source_frame_seq_valid && o.shard_index < o.shard_count && o.node_a == o.reporter_id && o.node_a != o.node_b;
      valid &= !o.rssi_valid && std::isnan(o.fp_rssi) && std::isnan(o.rx_rssi) && std::isfinite(o.distance_m);
      valid &= ids.emplace(m->driver_session_id, o.reporter_id, o.source_session_id, o.source_frame_seq, o.node_b).second;
    }
  });
  auto clock = node->create_subscription<rosgraph_msgs::msg::Clock>("/clock", qos, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr) { ++count[3]; });
  const auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && valid && count != expected && std::chrono::steady_clock::now() - start < std::chrono::seconds(40)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  valid &= count == expected;
  std::ofstream out(report);
  out << "{\"pass\":" << (valid ? "true" : "false") << ",\"counts\":[" << count[0] << "," << count[1] << "," << count[2] << "," << count[3] << "],\"localization\":\"NOT_RUN\"}\n";
  rclcpp::shutdown();
  return valid ? 0 : 1;
}
