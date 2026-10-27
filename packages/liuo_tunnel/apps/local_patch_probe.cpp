#include <liuo_tunnel/uwb/local_installation.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <array>
#include <cmath>

// Offline diagnostic only: no subscriptions, graph mutation, truth or priors.
int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::invalid_argument("usage: local_patch_probe fixture.json");
    std::ifstream metadata(argv[1]);
    const auto input = nlohmann::json::parse(metadata);
    Eigen::Matrix3d R;
    Eigen::Vector3d velocity;
    for (int i = 0; i < 3; ++i) {
      velocity[i] = input.at("velocity_body").at(i);
      for (int j = 0; j < 3; ++j) R(i, j) = input.at("rotation_body_to_estimated_world").at(i).at(j);
    }
    const gtsam::Rot3 gravity(R);
    const double sigma = input.at("range_sigma");
    std::ifstream stream(input.at("point_file").get<std::string>());
    if (!stream) throw std::runtime_error("fixture points missing");
    std::string line;
    std::getline(stream, line);
    std::vector<gtsam::Point3> points;
    while (std::getline(stream, line)) {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      gtsam::Point3 point;
      if (!(row >> point.x() >> point.y() >> point.z()) || !point.allFinite()) throw std::runtime_error("invalid fixture point");
      if (point.norm() <= 25) points.push_back(point);
    }
    nlohmann::json report = {{"scope", "offline production-geometry diagnostic, not independent acceptance"}, {"fixture", argv[1]}, {"points", points.size()}};
    try {
      const auto local = liuo_tunnel::local_installation_observation(points, gravity, velocity, sigma);
      report["observation"] = {{"result", "OBSERVED"}, {"sigma", local.sigma}};
    } catch (const std::exception& error) {
      report["observation"] = {{"result", "REJECTED"}, {"reason", error.what()}};
    }
    const auto wall = liuo_tunnel::body_wall_observation(points, gravity, sigma);
    const auto up = gravity.unrotate(gtsam::Vector3::UnitZ());
    auto normal = wall.normal;
    if (normal.dot(up.cross(velocity)) < 0) normal *= -1;
    const auto tangent = normal.cross(up).normalized().eval();
    const double center = normal.dot(wall.center);
    report["wall_bins"] = nlohmann::json::array();
    for (size_t side = 0; side < 2; ++side) {
      std::array<size_t, 6> bins{};
      double lo = INFINITY, hi = -INFINITY;
      for (const auto& point : points) {
        const double along = tangent.dot(point);
        if (along < -15 || along >= 15 || std::abs(normal.dot(point) - center - (side ? 1 : -1) * wall.leveled.width / 2) > .12) continue;
        ++bins[std::min<size_t>(5, size_t((along + 15) / 5))];
        lo = std::min(lo, along);
        hi = std::max(hi, along);
      }
      report["wall_bins"].push_back(
        {{"side", side},
         {"counts", bins},
         {"minimum", std::isfinite(lo) ? nlohmann::json(lo) : nlohmann::json(nullptr)},
         {"maximum", std::isfinite(hi) ? nlohmann::json(hi) : nlohmann::json(nullptr)}});
      try {
        const auto floor = liuo_tunnel::local_floor_patch(points, normal, tangent, up, center, wall.leveled.width, side, sigma);
        report["floors"].push_back({{"side", side}, {"minimum", floor.minimum_longitudinal}, {"maximum", floor.maximum_longitudinal}, {"variance", floor.variance}});
      } catch (const std::exception& error) {
        report["floors"].push_back({{"side", side}, {"error", error.what()}});
      }
    }
    std::cout << report.dump(2) << std::endl;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }
}
