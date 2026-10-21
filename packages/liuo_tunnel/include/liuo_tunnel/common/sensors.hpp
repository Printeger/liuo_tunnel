#pragma once
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <stdexcept>
namespace liuo_tunnel {
inline nlohmann::json load_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("missing config_file: " + path);
  auto j = nlohmann::json::parse(f);
  if (j.at("scan_model") != "snapshot") throw std::runtime_error("scan_model: only explicit snapshot is validated in phase one");
  if (j.at("uwb").at("mode") != "record_only" && j.at("uwb").at("mode") != "fuse") throw std::runtime_error("uwb.mode must be record_only or fuse");
  for (const char* key :
       {"sigma_acc_sample", "sigma_gyro_sample", "sigma_ba_density", "sigma_bg_density", "sigma_lidar_range", "initial_acc_bias_sigma", "initial_gyro_bias_sigma"}) {
    double v = j.at("sensors").at(key);
    if (!std::isfinite(v) || v <= 0) throw std::runtime_error(std::string("invalid sensors.") + key);
  }
  if (j.at("sensors").at("imu_rate_hz") != 200 || j.at("sensors").at("lidar_rate_hz") != 10) throw std::runtime_error("phase1 sensor rates must be 200/10 Hz");
  if (
    j.at("uwb").at("maximum_range_m") != 800 || j.at("uwb").at("sigma_m") != .10 || j.at("uwb").at("exchange_budget_hz") != 250 || j.at("uwb").at("ta_round_hz") != 20 ||
    j.at("uwb").at("aa_link_hz") != 1)
    throw std::runtime_error("unsupported uwb baseline parameters");
  const auto& odometry = j.at("odometry");
  if (odometry.value("max_keyframes", 15) < 1 || odometry.value("max_keyframes", 15) > 100) throw std::runtime_error("max_keyframes must be in [1,100]");
  double threshold = odometry.value("relinearize_threshold", .05);
  if (!std::isfinite(threshold) || threshold <= 0) throw std::runtime_error("invalid relinearize_threshold");
  return j;
}
inline Eigen::Vector3d vector3(const nlohmann::json& j) {
  if (!j.is_array() || j.size() != 3) throw std::runtime_error("extrinsic must contain three components");
  Eigen::Vector3d v(j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>());
  if (!v.allFinite()) throw std::runtime_error("nonfinite extrinsic");
  return v;
}
inline Eigen::Vector3d point_in_lidar(const Eigen::Isometry3d& T_WB, const Eigen::Isometry3d& T_BL, const Eigen::Vector3d& world_point) {
  return (T_WB * T_BL).inverse() * world_point;
}
inline Eigen::Vector3d tag_in_world(const Eigen::Vector3d& p, const Eigen::Matrix3d& R, const Eigen::Vector3d& tag) {
  return p + R * tag;
}
inline Eigen::Isometry3d lidar_extrinsic(const nlohmann::json& j) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = vector3(j.at("sensors").at("T_BL_translation"));
  double yaw = j.at("sensors").at("T_BL_yaw");
  if (!std::isfinite(yaw)) throw std::runtime_error("T_BL_yaw invalid");
  t.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return t;
}
}  // namespace liuo_tunnel
