#include <gtest/gtest.h>
#include <liuo_tunnel_sim/rail_model.hpp>
using namespace liuo_tunnel_sim;
TEST(Physics, ForceAndContinuousTrajectory) {
  EXPECT_NEAR(specific_force(motion(0)).z(), 9.80665, 1e-10);
  EXPECT_NEAR(motion(30).a.norm(), 0, 1e-10);
  EXPECT_NEAR(motion(30, 500).a.norm(), 0.05, 1e-10);
  for (double t = 0.01; t < 550; t += .01) {
    auto m = motion(t);
    auto before = motion(t - 1e-5), after = motion(t + 1e-5);
    EXPECT_LT((m.v - (after.p - before.p) / 2e-5).norm(), 1e-5);
    EXPECT_LE(m.a.norm(), 1.0);
    EXPECT_LE(m.v.norm(), 5.00001);
  }
}
TEST(Uwb, ThreeDimensionalBoundaryAndUnclippedTail) {
  Eigen::Vector3d a(12, 21, 33);
  Eigen::Vector3d u(.6, 0, .8);
  EXPECT_TRUE(reachable(a, a + 799.9 * u));
  EXPECT_TRUE(reachable(a, a + 800 * u));
  EXPECT_FALSE(reachable(a, a + 800.1 * u));
  int above = 0;
  for (int i = 0; i < 10000; i++)
    if (range(a, a + 800 * u, 0, 100, i) > 800) above++;
  EXPECT_GT(above, 4500);
  EXPECT_LT(above, 5500);
}
TEST(Uwb, GaussianAndIndependentStreams) {
  for (int stream : {100, 200}) {
    double sum = 0, sq = 0;
    const int n = 20000;
    for (int i = 0; i < n; i++) {
      const Eigen::Vector3d a = anchor(2), b = anchor(4);
      ASSERT_TRUE(line_of_sight(a, b));
      double e = range(a, b, 0, stream, i) - (a - b).norm();
      sum += e;
      sq += e * e;
    }
    double mean = sum / n, sd = std::sqrt((sq - n * mean * mean) / (n - 1));
    std::cout << "stream=" << stream << " N=" << n << " mean=" << mean << " sd=" << sd << std::endl;
    EXPECT_LT(std::abs(mean), 3 * .1 / std::sqrt(n));
    EXPECT_GE(sd, .095);
    EXPECT_LE(sd, .105);
  }
  double sample = normal(0, 3, 100);
  for (int i = 0; i < 20000; i++) normal(0, 4, i);
  EXPECT_EQ(sample, normal(0, 3, 100));
  EXPECT_NE(sample, normal(1, 3, 100));
}
TEST(Uwb, WorldLosIncludesCabinetsOutsideLidarCache) {
  EXPECT_FALSE(line_of_sight({0, 2.8, 2}, {400, 2.8, 2}));
  EXPECT_TRUE(line_of_sight(anchor(2), anchor(4)));
  EXPECT_FALSE(line_of_sight({100, 0, 1.2}, {100, 4, 2}));
  EXPECT_TRUE(line_of_sight({100, 0, 1.2}, {100, 2.8, 2}));
}

#include <Eigen/SVD>
TEST(Uwb, EveryLocalAnchorQuartetIsNoncoplanar) {
  for (int start = 0; start + 3 < anchor_count; start++) {
    Eigen::Matrix3d offsets;
    for (int j = 0; j < 3; j++) offsets.col(j) = anchor(start + j + 1) - anchor(start);
    auto spectrum = offsets.jacobiSvd().singularValues();
    std::cout << "anchor quartet " << start << " minimum singular value=" << spectrum[2] << std::endl;
    EXPECT_GT(spectrum[2], .01);
  }
}

TEST(Uwb, AnchorExchangeReservation) {
  int reserved = 0;
  for (uint64_t slot = 0; slot < 250; ++slot) reserved += prefer_anchor_exchange(slot);
  EXPECT_EQ(reserved, 25);
}

#include <nlohmann/json.hpp>
#include <fstream>
TEST(Uwb, OracleFixtureMatchesPhysicalLayout) {
  std::ifstream stream(LIUO_ORACLE_FIXTURE);
  ASSERT_TRUE(stream.good());
  nlohmann::json map;
  stream >> map;
  ASSERT_EQ(map.at("anchors").size(), anchor_count);
  for (int i = 0; i < anchor_count; ++i) {
    const auto& entry = map.at("anchors").at(i);
    ASSERT_EQ(entry.at("id").get<int>(), i);
    for (int k = 0; k < 3; ++k) EXPECT_DOUBLE_EQ(entry.at("mean_m").at(k).get<double>(), anchor(i)[k]);
  }
}
