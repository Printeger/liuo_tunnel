#include <gtest/gtest.h>
#include <liuo_tunnel/common/gauge_consistent_vgicp_gpu.hpp>
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
  for (int i = -12; i <= 12; i++)
    for (int j = -12; j <= 12; j++) {
      pts.emplace_back(i * .13, j * .13, .173, 1);
      pts.emplace_back(.237, i * .13, j * .13, 1);
      pts.emplace_back(i * .13, .321, j * .13, 1);
    }
  auto make = [](const std::vector<Eigen::Vector4d>& p) {
    gtsam_points::KdTree tree(p.data(), p.size());
    std::vector<int> neighbors(p.size() * 10);
    for (size_t i = 0; i < p.size(); i++) {
      size_t ids[10];
      double ds[10];
      tree.knn_search(p[i].data(), 10, ids, ds);
      for (int k = 0; k < 10; k++) neighbors[i * 10 + k] = ids[k];
    }
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(p);
    liuo_tunnel::CloudCovarianceEstimation cov;
    cloud->add_covs(cov.estimate(p, neighbors));
    return cloud;
  };
  auto target = make(pts);
  auto voxel = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(.5);
  voxel->insert(*target);
  for (auto& p : pts) p.x() -= .3;
  auto source = make(pts);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(0, 1, voxel, source);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
  gtsam::Values values;
  values.insert(0, gtsam::Pose3());
  values.insert(1, gtsam::Pose3(gtsam::Rot3(), {.25, .01, .01}));

  auto out = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
  EXPECT_NEAR(out.at<gtsam::Pose3>(1).translation().x(), .3, .03);
}
#include <gtsam/linear/HessianFactor.h>
TEST(Vgicp, UnsupportedPlaneDoesNotFabricateAxialInformation) {
  liuo_tunnel::CloudCovarianceEstimation original(1, false), guarded(1, true);
  Eigen::Matrix4d line = Eigen::Matrix4d::Zero();
  line(0, 0) = 1e-7;
  line(1, 1) = 2e-7;
  line(2, 2) = 1;
  auto old_cov = original.regularize(line), new_cov = guarded.regularize(line);
  EXPECT_GT((old_cov.topLeftCorner<3, 3>().inverse().norm()), new_cov.norm());
  std::vector<Eigen::Vector4d> points{{.15, .15, .15, 1}, {.15, .15, .25, 1}, {.15, .15, .35, 1}};
  auto block = [&](const Eigen::Matrix4d& cov) {
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
    cloud->add_covs(std::vector<Eigen::Matrix4d>(points.size(), cov));
    auto voxel = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(.5);
    voxel->insert(*cloud);
    gtsam_points::IntegratedVGICPFactor factor(0, 1, voxel, cloud);
    gtsam::Values values;
    values.insert(0, gtsam::Pose3());
    values.insert(1, gtsam::Pose3());
    gtsam::HessianFactor h(*factor.linearize(values));
    return h.hessianBlockDiagonal().at(1).eval();
  };
  auto old_h = block(old_cov), new_h = block(new_cov);
  EXPECT_GT(old_h(3, 3), new_h(3, 3) * 100);
  Eigen::Matrix4d plane = Eigen::Matrix4d::Zero();
  plane(0, 0) = .1;
  plane(1, 1) = .2;
  plane(2, 2) = 1e-7;
  EXPECT_NEAR(guarded.regularize(plane)(2, 2), .001, 1e-10);
}

#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
TEST(VgicpGPU, BinaryRawFactorRecoversTranslation) {
  gtsam_points::LinearizationHook::register_hook([] { return std::make_shared<gtsam_points::NonlinearFactorSetGPU>(); });
  std::vector<Eigen::Vector4d> points;
  for (int i = -12; i <= 12; i++)
    for (int j = -12; j <= 12; j++) {
      points.emplace_back(i * .13, j * .13, .173, 1);
      points.emplace_back(.237, i * .13, j * .13, 1);
      points.emplace_back(i * .13, .321, j * .13, 1);
    }
  auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cpu->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity() * .01));
  auto target = gtsam_points::PointCloudGPU::clone(*cpu);
  auto voxels = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  voxels->insert(*target);
  for (auto& p : points) p.x() -= .2;
  auto shifted = std::make_shared<gtsam_points::PointCloudCPU>(points);
  shifted->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity() * .01));
  auto source = gtsam_points::PointCloudGPU::clone(*shifted);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(0, 1, voxels, source);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
  gtsam::Values initial;
  initial.insert(0, gtsam::Pose3());
  initial.insert(1, gtsam::Pose3(gtsam::Rot3(), {.17, .01, .01}));
  auto result = gtsam_points::LevenbergMarquardtOptimizerExt(graph, initial).optimize();
  EXPECT_NEAR(result.at<gtsam::Pose3>(1).x(), .2, .03);
  EXPECT_EQ(graph.at(0)->keys().size(), 2);
}

TEST(VgicpGPU, BinaryFactorMustNotObserveCommonRigidMotion) {
  std::vector<Eigen::Vector4d> points;
  for (int i = -20; i <= 20; i++)
    for (int j = -12; j <= 12; j++) {
      points.emplace_back(i * .37, j * .13, .173, 1);
      points.emplace_back(.237, i * .37, j * .13, 1);
      points.emplace_back(i * .37, .321, j * .13, 1);
    }
  auto make = [](const std::vector<Eigen::Vector4d>& p) {
    auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(p);
    cpu->add_covs(std::vector<Eigen::Matrix4d>(p.size(), Eigen::Matrix4d::Identity() * .01));
    return gtsam_points::PointCloudGPU::clone(*cpu);
  };
  auto target = make(points);
  auto voxels = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  voxels->insert(*target);
  gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(.03, -.04, .02), {.2, .1, -.1});
  for (auto& point : points) point.head<3>() = delta.transformTo(gtsam::Point3(point.head<3>()));
  auto source = make(points);
  auto factor = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(0, 1, voxels, source);
  gtsam::Pose3 world(gtsam::Rot3::RzRyRx(.2, -.3, .5), {100, 20, 3});
  gtsam::Values values;
  values.insert(0, world);
  values.insert(1, world.compose(delta));
  gtsam_points::NonlinearFactorSetGPU batch;
  batch.add(factor);
  batch.linearize(values);
  gtsam::HessianFactor h(*factor->linearize(values));
  Eigen::Matrix<double, 12, 6> gauge;
  gauge.topRows<6>().setIdentity();
  gauge.bottomRows<6>() = delta.inverse().AdjointMap();
  auto information = h.information();
  double defect = (information * gauge).norm() / information.norm();
  std::cout << "GPU common-motion relative information defect=" << defect << std::endl;
  EXPECT_LT(defect, 1e-12);
  auto native = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(0, 1, voxels, source);
  gtsam_points::NonlinearFactorSetGPU reference;
  reference.add(native);
  reference.linearize(values);
  gtsam::HessianFactor original(*native->linearize(values));
  EXPECT_LT((h.hessianBlockDiagonal().at(1) - original.hessianBlockDiagonal().at(1)).norm(), 1e-10);
  EXPECT_LT((h.linearTerm(h.begin() + 1) - original.linearTerm(original.begin() + 1)).norm(), 1e-10);
  EXPECT_DOUBLE_EQ(h.constantTerm(), original.constantTerm());
  EXPECT_LT((h.information() - original.information()).norm() / original.information().norm(), 1e-6);
  EXPECT_NEAR(factor->error(values), native->error(values), 1e-8);
}

#include <liuo_tunnel/uwb/range_factors.hpp>
#include <liuo_tunnel/common/imu_integration.hpp>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
TEST(VgicpGPU, RawUwbPerturbationChangesSharedImuStatesAndLidarLinearization) {
  using namespace gtsam::symbol_shorthand;
  gtsam_points::LinearizationHook::register_hook([] { return std::make_shared<gtsam_points::NonlinearFactorSetGPU>(); });
  std::vector<Eigen::Vector4d> world;
  for (int i = -5; i <= 5; i++)
    for (int j = -5; j <= 5; j++) {
      world.emplace_back(i * .23, j * .23, .173, 1);
      world.emplace_back(.237, i * .23, j * .23, 1);
      world.emplace_back(i * .23, .321, j * .23, 1);
    }
  std::vector<gtsam_points::PointCloud::Ptr> clouds;
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxels;
  gtsam::Values initial;
  for (int i = 0; i < 3; i++) {
    auto points = world;
    for (auto& p : points) p.x() -= .2 * i;
    auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(points);
    cpu->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity() * .01));
    clouds.push_back(gtsam_points::PointCloudGPU::clone(*cpu));
    auto voxel = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
    voxel->insert(*clouds.back());
    voxels.push_back(voxel);
    initial.insert(X(i), gtsam::Pose3(gtsam::Rot3(), {.2 * i, 0, 0}));
    initial.insert(V(i), gtsam::Vector3(.2, 0, 0));
    initial.insert(B(i), gtsam::imuBias::ConstantBias());
  }
  gtsam::NonlinearFactorGraph shared;
  // Synthetic gauge and known initial velocity/bias, never a per-frame pose observation.
  shared.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), initial.at<gtsam::Pose3>(X(0)), gtsam::noiseModel::Isotropic::Sigma(6, .001));
  shared.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), initial.at<gtsam::Vector3>(V(0)), gtsam::noiseModel::Isotropic::Sigma(3, .1));
  shared.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, .001));
  liuo_tunnel::IMUIntegration imu;
  for (int i = 0; i <= 400; i++) imu.insert_imu(1 + .005 * i, {0, 0, 9.80665}, {0, 0, 0});
  for (int i = 1; i < 3; i++) {
    int integrated;
    imu.integrate_imu(i, i + 1, gtsam::imuBias::ConstantBias(), &integrated);
    ASSERT_EQ(integrated, 200);
    shared.emplace_shared<gtsam::ImuFactor>(X(i - 1), V(i - 1), X(i), V(i), B(i - 1), imu.integrated_measurements());
    shared.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(B(i - 1), B(i), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, .001));
    for (int j = 0; j < i; j++) shared.emplace_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(X(j), X(i), voxels[j], clouds[i]);
  }
  const gtsam::Point3 anchor(4, 3, 2), lever(.12, -.08, .21);
  const double measured = (gtsam::Point3(.3, 0, 0) + lever - anchor).norm();
  auto solve = [&](double perturbation) {
    auto graph = shared;
    graph.emplace_shared<liuo_tunnel::TagAnchorRangeFactor>(X(1), X(2), std::nullopt, anchor, lever, .5, measured + perturbation, .1);
    gtsam_points::LevenbergMarquardtExtParams parameters;
    parameters.maxIterations = 30;
    parameters.relativeErrorTol = 1e-9;
    return gtsam::Values(gtsam_points::LevenbergMarquardtOptimizerExt(graph, initial, parameters).optimize());
  };
  auto nominal = solve(0), perturbed = solve(.3);
  double dx = (nominal.at<gtsam::Pose3>(X(2)).translation() - perturbed.at<gtsam::Pose3>(X(2)).translation()).norm();
  double dv = (nominal.at<gtsam::Vector3>(V(2)) - perturbed.at<gtsam::Vector3>(V(2))).norm();
  double db = (nominal.at<gtsam::imuBias::ConstantBias>(B(1)).vector() - perturbed.at<gtsam::imuBias::ConstantBias>(B(1)).vector()).norm();
  EXPECT_GT(dx, 1e-5);
  EXPECT_GT(dv, 1e-6);
  EXPECT_GT(db, 1e-10);
  auto next_lidar = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(X(0), X(2), voxels[0], clouds[2]);
  auto linearized = [&](const gtsam::Values& values) {
    gtsam_points::NonlinearFactorSetGPU batch;
    batch.add(next_lidar);
    batch.linearize(values);
    return gtsam::HessianFactor(*next_lidar->linearize(values)).augmentedInformation();
  };
  double linearization_change = (linearized(nominal) - linearized(perturbed)).norm();
  EXPECT_GT(linearization_change, 1e-5);
  std::cout << "single raw UWB +0.3m: dx=" << dx << " dv=" << dv << " db=" << db << " next VGICP augmented-information change=" << linearization_change << std::endl;
}

TEST(VgicpGPU, ExplicitTangentVarianceChangesActualFactorInformation) {
  std::vector<Eigen::Vector4d> points;
  for (int i = -15; i <= 15; i++)
    for (int j = -15; j <= 15; j++) points.emplace_back(i * .13, j * .13, .173, 1);
  Eigen::Matrix4d empirical = Eigen::Matrix4d::Zero();
  empirical(0, 0) = .1;
  empirical(1, 1) = .2;
  empirical(2, 2) = 1e-7;
  auto information = [&](double tangent) {
    liuo_tunnel::CloudCovarianceEstimation estimator(1, true, tangent);
    auto covariance = estimator.regularize(empirical);
    EXPECT_NEAR(covariance(2, 2), .001, 1e-12);
    auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(points);
    cpu->add_covs(std::vector<Eigen::Matrix4d>(points.size(), covariance));
    auto source = gtsam_points::PointCloudGPU::clone(*cpu);
    auto target = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
    target->insert(*source);
    auto factor = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(0, 1, target, source);
    gtsam::Values values;
    values.insert(0, gtsam::Pose3());
    values.insert(1, gtsam::Pose3());
    gtsam_points::NonlinearFactorSetGPU batch;
    batch.add(factor);
    batch.linearize(values);
    gtsam::HessianFactor h(*factor->linearize(values));
    return h.hessianBlockDiagonal().at(1).eval();
  };
  auto original = information(1), weakened = information(100);
  EXPECT_NEAR(weakened(3, 3) / original(3, 3), .01, 1e-6);
  EXPECT_NEAR(weakened(4, 4) / original(4, 4), .01, 1e-6);
  EXPECT_NEAR(weakened(5, 5) / original(5, 5), 1.0, 1e-5);
  EXPECT_THROW(liuo_tunnel::CloudCovarianceEstimation(1, true, 0), std::invalid_argument);
  EXPECT_THROW(liuo_tunnel::CloudCovarianceEstimation(1, true, 101), std::invalid_argument);
}

TEST(VgicpGPU, PointCountNormalizationScalesCostGradientAndInformationTogether) {
  std::vector<Eigen::Vector4d> points;
  for (int i = -10; i <= 10; i++)
    for (int j = -10; j <= 10; j++) points.emplace_back(i * .13, j * .13, .173, 1);
  auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cpu->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity()));
  auto source = gtsam_points::PointCloudGPU::clone(*cpu);
  auto target = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  target->insert(*source);
  const double weight = 1.0 / points.size();
  auto original = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(0, 1, target, source);
  auto normalized = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(0, 1, target, source, nullptr, nullptr, weight);
  auto cloned = normalized->clone();
  gtsam::Values values;
  values.insert(0, gtsam::Pose3());
  values.insert(1, gtsam::Pose3(gtsam::Rot3(), {.01, .02, .03}));
  gtsam_points::NonlinearFactorSetGPU batch;
  batch.add(original);
  batch.add(normalized);
  batch.add(cloned);
  batch.linearize(values);
  batch.error(values);
  gtsam::HessianFactor h(*original->linearize(values)), scaled(*normalized->linearize(values)), copy(*cloned->linearize(values));
  EXPECT_GT(original->error(values), 0);
  EXPECT_NEAR(normalized->error(values), weight * original->error(values), 1e-12);
  EXPECT_LT((scaled.augmentedInformation() - weight * h.augmentedInformation()).norm(), 1e-10);
  EXPECT_LT((copy.augmentedInformation() - scaled.augmentedInformation()).norm(), 1e-10);
  EXPECT_NEAR(cloned->error(values), normalized->error(values), 1e-12);
}
