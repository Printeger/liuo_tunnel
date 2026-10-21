// Adapted from IAP src/uav_simulator/local_sensing/include/local_sensing/first_hit_lidar_renderer.hpp @ fbccae168; renderer retained, project namespace changed.
#ifndef LIUO_TUNNEL_SIM__FIRST_HIT_LIDAR_RENDERER_HPP_
#define LIUO_TUNNEL_SIM__FIRST_HIT_LIDAR_RENDERER_HPP_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace liuo_tunnel_sim
{

struct FirstHitLidarConfig
{
  int horizontal_samples = 512;
  int vertical_samples = 40;
  double horizontal_fov_deg = 360.0;
  double vertical_min_deg = -7.0;
  double vertical_max_deg = 52.0;
  double min_range_m = 0.1;
  double max_range_m = 10.0;
  double world_voxel_resolution_m = 0.1;
};

struct FirstHitLidarStats
{
  std::size_t ray_count = 0;
  std::size_t hit_count = 0;
  std::size_t dda_voxel_visits = 0;
  double render_latency_ms = 0.0;
};

enum class FirstHitLidarBeamOutcome : std::uint8_t
{
  INVALID = 0,
  HIT = 1,
  NO_RETURN = 2,
};

struct FirstHitLidarBeam
{
  Eigen::Vector3d direction_sensor = Eigen::Vector3d::Zero();
  FirstHitLidarBeamOutcome outcome = FirstHitLidarBeamOutcome::INVALID;
  double range_m = 0.0;
};

struct FirstHitLidarScan
{
  pcl::PointCloud<pcl::PointXYZ> hits;
  std::vector<FirstHitLidarBeam> beams;
  FirstHitLidarStats stats;
};

class FirstHitLidarRenderer
{
public:
  explicit FirstHitLidarRenderer(FirstHitLidarConfig config);
  ~FirstHitLidarRenderer();
  FirstHitLidarRenderer(FirstHitLidarRenderer &&) noexcept;
  FirstHitLidarRenderer & operator=(FirstHitLidarRenderer &&) noexcept;
  FirstHitLidarRenderer(const FirstHitLidarRenderer &) = delete;
  FirstHitLidarRenderer & operator=(const FirstHitLidarRenderer &) = delete;

  bool loadWorld(
    const pcl::PointCloud<pcl::PointXYZ> & world,
    std::string * reason = nullptr);
  FirstHitLidarScan render(const Eigen::Isometry3d & sensor_pose_w) const;
  const FirstHitLidarConfig & config() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace liuo_tunnel_sim

#endif  // LIUO_TUNNEL_SIM__FIRST_HIT_LIDAR_RENDERER_HPP_
