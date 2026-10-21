#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>
namespace liuo_tunnel_sim {
// Corridor geometry adapted from IAP demo11_corridor_map_publisher.cpp.
inline pcl::PointCloud<pcl::PointXYZ> rail_world(int block) {
  pcl::PointCloud<pcl::PointXYZ> p;
  const int lo=std::max(-1,block-3)*200, hi=std::min(121,block+4)*200;
  for(int ix=lo;ix<hi;ix++) {
   // Occupancy samples must lie inside voxels, not on float-rounded boundaries.
   // Boundary samples left empty columns (e.g. x=95.15), making solid walls porous.
   float xx=(ix+.5f)*.1f;
   for(int iz=0;iz<50;iz++) { p.emplace_back(xx,-3.05f,(iz+.5f)*.1f); p.emplace_back(xx,3.05f,(iz+.5f)*.1f); }
   for(int iy=-30;iy<30;iy++) {p.emplace_back(xx,(iy+.5f)*.1f,-.05f); p.emplace_back(xx,(iy+.5f)*.1f,5.05f);}
   for(float yy : {-.7175f,.7175f}) p.emplace_back(xx,yy,.15f);
   if(ix%6==0) for(int iy=-12;iy<=12;iy++) p.emplace_back(xx,(iy+.5f)*.1f,.05f);
   // Nonperiodic cabinets only at the entrance; adapted corridor surface generation from IAP demo11.
   if(xx>=0 && xx<50) for(int k=0;k<8;k++) {
    float center=3.1f+k*5.7f+std::sin(k*1.7f);
    if(std::abs(xx-center)<.4) for(int iz=3;iz<23;iz++) for(int iy=22;iy<30;iy++) p.emplace_back(xx,(k%2?1:-1)*(iy+.5f)*.1f,(iz+.5f)*.1f);
   }
  }
return p;
}
}
