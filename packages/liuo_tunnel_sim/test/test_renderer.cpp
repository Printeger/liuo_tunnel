#include <liuo_tunnel_sim/first_hit_lidar_renderer.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{

liuo_tunnel_sim::FirstHitLidarConfig oneForwardRay()
{
  liuo_tunnel_sim::FirstHitLidarConfig config;
  config.horizontal_samples = 1;
  config.vertical_samples = 1;
  config.horizontal_fov_deg = 0.0;
  config.vertical_min_deg = 0.0;
  config.vertical_max_deg = 0.0;
  config.min_range_m = 0.1;
  config.max_range_m = 10.0;
  config.world_voxel_resolution_m = 0.1;
  return config;
}

TEST(FirstHitLidarRenderer, OccluderSuppressesEveryPointBehindIt)
{
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(2.05F, 0.05F, 0.05F));
  world.push_back(pcl::PointXYZ(4.05F, 0.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));

  const auto scan = renderer.render(Eigen::Isometry3d::Identity());

  ASSERT_EQ(scan.hits.size(), 1u);
  EXPECT_NEAR(scan.hits.front().x, 2.0, 0.11);
  EXPECT_LT(scan.hits.front().x, 3.0);
  EXPECT_EQ(scan.stats.ray_count, 1u);
  EXPECT_EQ(scan.stats.hit_count, 1u);
  ASSERT_EQ(scan.beams.size(), 1u);
  EXPECT_EQ(scan.beams.front().outcome,
            liuo_tunnel_sim::FirstHitLidarBeamOutcome::HIT);
  EXPECT_NEAR(scan.beams.front().range_m, 2.0, 0.11);
}

TEST(FirstHitLidarRenderer, NoReturnProducesNoSyntheticPoint)
{
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(0.05F, 2.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));

  const auto scan = renderer.render(Eigen::Isometry3d::Identity());

  EXPECT_TRUE(scan.hits.empty());
  EXPECT_EQ(scan.stats.ray_count, 1u);
  EXPECT_EQ(scan.stats.hit_count, 0u);
  ASSERT_EQ(scan.beams.size(), 1u);
  EXPECT_EQ(scan.beams.front().outcome,
            liuo_tunnel_sim::FirstHitLidarBeamOutcome::NO_RETURN);
  EXPECT_DOUBLE_EQ(scan.beams.front().range_m,
                   oneForwardRay().max_range_m);
}

TEST(FirstHitLidarRenderer, InvalidPoseProducesExplicitInvalidBeams)
{
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(oneForwardRay());
  ASSERT_TRUE(renderer.loadWorld({}));
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();

  const auto scan = renderer.render(invalid);

  ASSERT_EQ(scan.beams.size(), 1u);
  EXPECT_EQ(scan.beams.front().outcome,
            liuo_tunnel_sim::FirstHitLidarBeamOutcome::INVALID);
}

TEST(FirstHitLidarRenderer, SensorPoseRotatesTheRegularSphericalScan)
{
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(0.05F, 2.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(
    0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto first = renderer.render(pose);
  const auto second = renderer.render(pose);

  ASSERT_EQ(first.hits.size(), 1u);
  ASSERT_EQ(second.hits.size(), first.hits.size());
  EXPECT_NEAR(first.hits.front().y, 2.0, 0.11);
  EXPECT_FLOAT_EQ(first.hits.front().x, second.hits.front().x);
  EXPECT_FLOAT_EQ(first.hits.front().y, second.hits.front().y);
  EXPECT_FLOAT_EQ(first.hits.front().z, second.hits.front().z);
}

TEST(FirstHitLidarRenderer, FullAzimuthAndVerticalEndpointsAreRendered)
{
  liuo_tunnel_sim::FirstHitLidarConfig azimuth_config = oneForwardRay();
  azimuth_config.horizontal_samples = 4;
  azimuth_config.horizontal_fov_deg = 360.0;
  azimuth_config.world_voxel_resolution_m = 0.05;
  liuo_tunnel_sim::FirstHitLidarRenderer azimuth_renderer(azimuth_config);
  pcl::PointCloud<pcl::PointXYZ> azimuth_world;
  for (const double degrees : {0.0, 90.0, 180.0, 270.0}) {
    const double radians = degrees * M_PI / 180.0;
    azimuth_world.push_back(pcl::PointXYZ(
        static_cast<float>(2.0 * std::cos(radians)),
        static_cast<float>(2.0 * std::sin(radians)), 0.0F));
  }
  ASSERT_TRUE(azimuth_renderer.loadWorld(azimuth_world));
  EXPECT_EQ(
    azimuth_renderer.render(Eigen::Isometry3d::Identity()).hits.size(), 4u);

  liuo_tunnel_sim::FirstHitLidarConfig vertical_config = oneForwardRay();
  vertical_config.vertical_samples = 2;
  vertical_config.vertical_min_deg = -7.0;
  vertical_config.vertical_max_deg = 52.0;
  vertical_config.world_voxel_resolution_m = 0.05;
  liuo_tunnel_sim::FirstHitLidarRenderer vertical_renderer(vertical_config);
  pcl::PointCloud<pcl::PointXYZ> vertical_world;
  for (const double degrees : {-7.0, 52.0}) {
    const double radians = degrees * M_PI / 180.0;
    vertical_world.push_back(pcl::PointXYZ(
        static_cast<float>(2.0 * std::cos(radians)), 0.0F,
        static_cast<float>(2.0 * std::sin(radians))));
  }
  ASSERT_TRUE(vertical_renderer.loadWorld(vertical_world));
  const auto vertical_scan = vertical_renderer.render(
    Eigen::Isometry3d::Identity());
  ASSERT_EQ(vertical_scan.hits.size(), 2u);
  std::vector<double> elevations;
  for (const auto & hit : vertical_scan.hits) {
    elevations.push_back(std::atan2(
        hit.z, std::hypot(hit.x, hit.y)) * 180.0 / M_PI);
  }
  std::sort(elevations.begin(), elevations.end());
  EXPECT_NEAR(elevations.front(), -7.0, 0.1);
  EXPECT_NEAR(elevations.back(), 52.0, 0.1);
}

TEST(FirstHitLidarRenderer, MinAndMaxRangeExcludeOutOfRangeVoxels)
{
  auto config = oneForwardRay();
  config.min_range_m = 0.1;
  config.max_range_m = 1.0;
  config.world_voxel_resolution_m = 0.01;
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(config);
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(0.055F, 0.005F, 0.005F));
  world.push_back(pcl::PointXYZ(1.105F, 0.005F, 0.005F));
  ASSERT_TRUE(renderer.loadWorld(world));
  EXPECT_TRUE(renderer.render(Eigen::Isometry3d::Identity()).hits.empty());

  world.push_back(pcl::PointXYZ(0.505F, 0.005F, 0.005F));
  ASSERT_TRUE(renderer.loadWorld(world));
  const auto scan = renderer.render(Eigen::Isometry3d::Identity());
  ASSERT_EQ(scan.hits.size(), 1u);
  EXPECT_NEAR(scan.hits.front().x, 0.5, 0.02);
}

TEST(FirstHitLidarRenderer, ForestSizedWorldMeetsTenHertzRenderBudget)
{
  liuo_tunnel_sim::FirstHitLidarConfig config;
  liuo_tunnel_sim::FirstHitLidarRenderer renderer(config);
  pcl::PointCloud<pcl::PointXYZ> world;
  for (double x = -21.0; x <= 21.0; x += 0.2) {
    for (double y = -11.0; y <= 11.0; y += 0.2) {
      world.push_back(pcl::PointXYZ(
          static_cast<float>(x), static_cast<float>(y), 0.05F));
    }
  }
  for (double x = -18.0; x <= 18.0; x += 2.0) {
    for (double y = -8.0; y <= 8.0; y += 2.0) {
      if (std::abs(y) < 2.0) {
        continue;
      }
      for (double z = 0.1; z <= 6.0; z += 0.1) {
        world.push_back(pcl::PointXYZ(
            static_cast<float>(x), static_cast<float>(y),
            static_cast<float>(z)));
      }
    }
  }
  ASSERT_TRUE(renderer.loadWorld(world));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-18.0, 0.0, 1.5);

  static_cast<void>(renderer.render(pose));
  std::vector<double> latency_ms;
  for (int frame = 0; frame < 12; ++frame) {
    const auto scan = renderer.render(pose);
    EXPECT_EQ(scan.stats.ray_count, 20480u);
    latency_ms.push_back(scan.stats.render_latency_ms);
  }
  std::sort(latency_ms.begin(), latency_ms.end());
  const std::size_t p95_index = static_cast<std::size_t>(
      0.95 * static_cast<double>(latency_ms.size() - 1));
  EXPECT_LT(latency_ms[p95_index], 80.0);
}

}  // namespace
