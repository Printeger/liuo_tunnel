#include <gtest/gtest.h>
#include <liuo_tunnel/common/cloud_covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
TEST(Vgicp, VariablePoseFactorRecoversTranslatedGeometry) {
// Offset the planes away from voxel boundaries: CPU VGICP uses same-cell lookup.
 // A plane exactly on a boundary can have no correspondences for one sign of prediction error.
 std::vector<Eigen::Vector4d> pts;
 for(int i=-12;i<=12;i++)for(int j=-12;j<=12;j++) {
  pts.emplace_back(i*.13,j*.13,.173,1);pts.emplace_back(.237,i*.13,j*.13,1);pts.emplace_back(i*.13,.321,j*.13,1);
 }
 auto make=[](const std::vector<Eigen::Vector4d>& p){
  gtsam_points::KdTree tree(p.data(),p.size());std::vector<int> neighbors(p.size()*10);
  for(size_t i=0;i<p.size();i++){size_t ids[10];double ds[10];tree.knn_search(p[i].data(),10,ids,ds);for(int k=0;k<10;k++)neighbors[i*10+k]=ids[k];}
  auto cloud=std::make_shared<gtsam_points::PointCloudCPU>(p);liuo_tunnel::CloudCovarianceEstimation cov;cloud->add_covs(cov.estimate(p,neighbors));return cloud;
 };
 auto target=make(pts);auto voxel=std::make_shared<gtsam_points::GaussianVoxelMapCPU>(.5);voxel->insert(*target);
 for(auto& p:pts)p.x()-=.3;auto source=make(pts);
 gtsam::NonlinearFactorGraph graph;graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(0,1,voxel,source);
 graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0,gtsam::Pose3(),gtsam::noiseModel::Isotropic::Sigma(6,1e-5));
 gtsam::Values values;values.insert(0,gtsam::Pose3());values.insert(1,gtsam::Pose3(gtsam::Rot3(),{.25,.01,.01}));

 auto out=gtsam::LevenbergMarquardtOptimizer(graph,values).optimize();
 EXPECT_NEAR(out.at<gtsam::Pose3>(1).translation().x(),.3,.03);
}
