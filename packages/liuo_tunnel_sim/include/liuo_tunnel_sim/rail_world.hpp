#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <liuo_tunnel_sim/rail_model.hpp>
#include <algorithm>
namespace liuo_tunnel_sim {
// Corridor geometry adapted from IAP demo11_corridor_map_publisher.cpp.
inline pcl::PointCloud<pcl::PointXYZ> rail_world(int block) {
  pcl::PointCloud<pcl::PointXYZ> p;
  const int lo = std::max(0, block - 3) * 200, hi = std::min(120, block + 4) * 200;
  for (int ix = lo; ix < hi; ix++) {
    // Occupancy samples must lie inside voxels, not on float-rounded boundaries.
    // Boundary samples left empty columns (e.g. x=95.15), making solid walls porous.
    float xx = (ix + .5f) * .1f;
    for (int iz = 0; iz < 50; iz++) {
      p.emplace_back(xx, -3.05f, (iz + .5f) * .1f);
      p.emplace_back(xx, 3.05f, (iz + .5f) * .1f);
    }
    for (int iy = -30; iy < 30; iy++) {
      p.emplace_back(xx, (iy + .5f) * .1f, -.05f);
      p.emplace_back(xx, (iy + .5f) * .1f, 5.05f);
    }

  }
  return p;
}
}  // namespace liuo_tunnel_sim
