// Adapted from IAP src/uav_simulator/local_sensing/src/first_hit_lidar_renderer.cpp @ fbccae168; renderer retained, project namespace changed.
#include <liuo_tunnel_sim/first_hit_lidar_renderer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace liuo_tunnel_sim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDirectionEpsilon = 1.0e-12;

struct VoxelIndex
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & index) const
  {
    std::size_t seed = 0xcbf29ce484222325ULL;
    const auto mix = [&seed](const int value) {
        seed ^= static_cast<std::uint32_t>(value);
        seed *= 0x100000001b3ULL;
      };
    mix(index.x);
    mix(index.y);
    mix(index.z);
    return seed;
  }
};

VoxelIndex voxelIndex(const Eigen::Vector3d & point, const double resolution)
{
  return {
    static_cast<int>(std::floor(point.x() / resolution)),
    static_cast<int>(std::floor(point.y() / resolution)),
    static_cast<int>(std::floor(point.z() / resolution))};
}

bool finitePoint(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

}  // namespace

struct FirstHitLidarRenderer::Impl
{
  explicit Impl(FirstHitLidarConfig input) : config(std::move(input)) {}

  FirstHitLidarConfig config;
  std::unordered_set<VoxelIndex, VoxelIndexHash> occupied;
  std::vector<Eigen::Vector3d> directions_sensor;
  bool valid_config = false;
  bool world_loaded = false;

  void buildDirections()
  {
    directions_sensor.clear();
    directions_sensor.reserve(
      static_cast<std::size_t>(config.horizontal_samples) *
      static_cast<std::size_t>(config.vertical_samples));
    for (int vertical = 0; vertical < config.vertical_samples; ++vertical) {
      const double vertical_fraction = config.vertical_samples == 1 ? 0.0 :
        static_cast<double>(vertical) /
        static_cast<double>(config.vertical_samples - 1);
      const double elevation_deg = config.vertical_samples == 1 ?
        0.5 * (config.vertical_min_deg + config.vertical_max_deg) :
        config.vertical_min_deg + vertical_fraction *
        (config.vertical_max_deg - config.vertical_min_deg);
      const double elevation = elevation_deg * kPi / 180.0;
      for (int horizontal = 0; horizontal < config.horizontal_samples;
        ++horizontal)
      {
        double azimuth_deg = 0.0;
        if (config.horizontal_samples > 1) {
          if (config.horizontal_fov_deg >= 360.0 - 1.0e-9) {
            azimuth_deg = config.horizontal_fov_deg *
              static_cast<double>(horizontal) /
              static_cast<double>(config.horizontal_samples);
          } else {
            azimuth_deg = -0.5 * config.horizontal_fov_deg +
              config.horizontal_fov_deg * static_cast<double>(horizontal) /
              static_cast<double>(config.horizontal_samples - 1);
          }
        }
        const double azimuth = azimuth_deg * kPi / 180.0;
        const double cos_elevation = std::cos(elevation);
        directions_sensor.emplace_back(
          cos_elevation * std::cos(azimuth),
          cos_elevation * std::sin(azimuth), std::sin(elevation));
      }
    }
  }

  bool firstHit(
    const Eigen::Vector3d & origin, const Eigen::Vector3d & direction,
    Eigen::Vector3d * hit, std::size_t * visits) const
  {
    const double resolution = config.world_voxel_resolution_m;
    const Eigen::Vector3d start = origin + direction * config.min_range_m;
    VoxelIndex index = voxelIndex(start, resolution);
    Eigen::Vector3i step;
    Eigen::Vector3d t_max;
    Eigen::Vector3d t_delta;
    for (int axis = 0; axis < 3; ++axis) {
      if (direction[axis] > kDirectionEpsilon) {
        step[axis] = 1;
        const double boundary =
          (static_cast<double>((axis == 0 ? index.x :
          axis == 1 ? index.y : index.z) + 1)) * resolution;
        t_max[axis] = (boundary - origin[axis]) / direction[axis];
        t_delta[axis] = resolution / direction[axis];
      } else if (direction[axis] < -kDirectionEpsilon) {
        step[axis] = -1;
        const double boundary =
          static_cast<double>(axis == 0 ? index.x :
          axis == 1 ? index.y : index.z) * resolution;
        t_max[axis] = (boundary - origin[axis]) / direction[axis];
        t_delta[axis] = -resolution / direction[axis];
      } else {
        step[axis] = 0;
        t_max[axis] = std::numeric_limits<double>::infinity();
        t_delta[axis] = std::numeric_limits<double>::infinity();
      }
    }

    double entered_at = config.min_range_m;
    while (entered_at <= config.max_range_m + 1.0e-9) {
      ++(*visits);
      if (occupied.find(index) != occupied.end()) {
        const double epsilon = std::min(1.0e-6, 0.01 * resolution);
        *hit = origin + direction * std::min(
          config.max_range_m, entered_at + epsilon);
        return true;
      }
      int axis = 0;
      if (t_max.y() < t_max.x()) {
        axis = 1;
      }
      if (t_max.z() < t_max[axis]) {
        axis = 2;
      }
      entered_at = t_max[axis];
      if (!std::isfinite(entered_at) || entered_at > config.max_range_m) {
        break;
      }
      if (axis == 0) {
        index.x += step.x();
      } else if (axis == 1) {
        index.y += step.y();
      } else {
        index.z += step.z();
      }
      t_max[axis] += t_delta[axis];
    }
    return false;
  }
};

FirstHitLidarRenderer::FirstHitLidarRenderer(FirstHitLidarConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
  const auto & value = impl_->config;
  impl_->valid_config = value.horizontal_samples > 0 &&
    value.vertical_samples > 0 && std::isfinite(value.horizontal_fov_deg) &&
    value.horizontal_fov_deg >= 0.0 && value.horizontal_fov_deg <= 360.0 &&
    std::isfinite(value.vertical_min_deg) &&
    std::isfinite(value.vertical_max_deg) &&
    value.vertical_min_deg <= value.vertical_max_deg &&
    std::isfinite(value.min_range_m) && value.min_range_m >= 0.0 &&
    std::isfinite(value.max_range_m) &&
    value.max_range_m > value.min_range_m &&
    std::isfinite(value.world_voxel_resolution_m) &&
    value.world_voxel_resolution_m > 0.0;
  if (impl_->valid_config) {
    impl_->buildDirections();
  }
}

FirstHitLidarRenderer::~FirstHitLidarRenderer() = default;
FirstHitLidarRenderer::FirstHitLidarRenderer(
  FirstHitLidarRenderer &&) noexcept = default;
FirstHitLidarRenderer & FirstHitLidarRenderer::operator=(
  FirstHitLidarRenderer &&) noexcept = default;

bool FirstHitLidarRenderer::loadWorld(
  const pcl::PointCloud<pcl::PointXYZ> & world, std::string * reason)
{
  impl_->occupied.clear();
  impl_->world_loaded = false;
  if (!impl_->valid_config) {
    if (reason) {
      *reason = "invalid_config";
    }
    return false;
  }
  for (const auto & point : world.points) {
    if (!finitePoint(point)) {
      continue;
    }
    impl_->occupied.insert(voxelIndex(
      Eigen::Vector3d(point.x, point.y, point.z),
      impl_->config.world_voxel_resolution_m));
  }
  impl_->world_loaded = true;
  if (reason) {
    *reason = "ok";
  }
  return true;
}

FirstHitLidarScan FirstHitLidarRenderer::render(
  const Eigen::Isometry3d & sensor_pose_w) const
{
  const auto started = std::chrono::steady_clock::now();
  FirstHitLidarScan scan;
  scan.stats.ray_count = impl_->directions_sensor.size();
  scan.hits.reserve(scan.stats.ray_count);
  scan.beams.reserve(scan.stats.ray_count);
  if (impl_->world_loaded && sensor_pose_w.matrix().allFinite()) {
    const Eigen::Vector3d origin = sensor_pose_w.translation();
    for (const auto & direction_sensor : impl_->directions_sensor) {
      const Eigen::Vector3d direction =
        (sensor_pose_w.linear() * direction_sensor).normalized();
      Eigen::Vector3d hit;
      if (impl_->firstHit(
          origin, direction, &hit, &scan.stats.dda_voxel_visits))
      {
        scan.hits.push_back(pcl::PointXYZ(
          static_cast<float>(hit.x()), static_cast<float>(hit.y()),
          static_cast<float>(hit.z())));
        scan.beams.push_back(FirstHitLidarBeam{
          direction_sensor, FirstHitLidarBeamOutcome::HIT,
          (hit - origin).norm()});
      } else {
        scan.beams.push_back(FirstHitLidarBeam{
          direction_sensor, FirstHitLidarBeamOutcome::NO_RETURN,
          impl_->config.max_range_m});
      }
    }
  } else {
    for (const auto & direction_sensor : impl_->directions_sensor) {
      scan.beams.push_back(FirstHitLidarBeam{
        direction_sensor, FirstHitLidarBeamOutcome::INVALID, 0.0});
    }
  }
  scan.hits.width = static_cast<std::uint32_t>(scan.hits.size());
  scan.hits.height = 1;
  scan.hits.is_dense = true;
  scan.stats.hit_count = scan.hits.size();
  scan.stats.render_latency_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  return scan;
}

const FirstHitLidarConfig & FirstHitLidarRenderer::config() const
{
  return impl_->config;
}

}  // namespace liuo_tunnel_sim
