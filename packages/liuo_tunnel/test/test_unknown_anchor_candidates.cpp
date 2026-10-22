#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/unknown_anchor_candidates.hpp>
using namespace liuo_tunnel;
namespace {
std::vector<TagRangeSeedObservation> observations(const Eigen::Vector3d& anchor, bool excite) {
  std::vector<TagRangeSeedObservation> samples;
  for (int i = 0; i < 32; ++i) {
    Eigen::Vector3d p(i, .3, excite ? .6 * std::sin(i * .2) : 0);
    samples.push_back({p, (p - anchor).norm()});
  }
  return samples;
}
}
TEST(UnknownAnchorSeeds, RetainsHeightMirrorsWithoutCoordinatesOrOrder) {
  Eigen::Vector3d anchor(190, 2.8, 3.4);
  auto samples = observations(anchor, false);
  auto seeds = unknown_anchor_seeds(samples, {0, 1}, 0, 2.8);
  ASSERT_EQ(seeds.reason, "UNRESOLVED_MIRROR_PAIR");
  ASSERT_EQ(seeds.points.size(), 2);
  EXPECT_LT(std::min((seeds.points[0] - anchor).norm(), (seeds.points[1] - anchor).norm()), 1e-8);
  for (const auto& point : seeds.points)
    for (const auto& sample : samples) EXPECT_NEAR((sample.tag - point).norm(), sample.range, 1e-9);
}
TEST(UnknownAnchorSeeds, ActualVerticalExcitationProvidesUniqueSeedNotPrior) {
  Eigen::Vector3d anchor(-120, -2.7, -1.3);
  auto seeds = unknown_anchor_seeds(observations(anchor, true), {0, 1}, 0, -2.7);
  ASSERT_EQ(seeds.reason, "MOTION_PLANE_SEED");
  ASSERT_EQ(seeds.points.size(), 1);
  EXPECT_LT((seeds.points.front() - anchor).norm(), 1e-8);
}
TEST(UnknownAnchorSeeds, DoesNotInventSeedForStaticOrOverCapacityInput) {
  std::vector<TagRangeSeedObservation> samples(10, {Eigen::Vector3d::Zero(), 200});
  EXPECT_TRUE(unknown_anchor_seeds(samples, {0, 1}, 0, 2.8).points.empty());
  samples.resize(513, samples.front());
  EXPECT_EQ(unknown_anchor_seeds(samples, {0, 1}, 0, 2.8).reason, "SEED_CAPACITY_EXCEEDED");
}
TEST(UnknownAnchorSeeds, TranslationAndYawGaugeDoNotChangePhysicalSeeds) {
  Eigen::Vector3d anchor(190, 2.8, 3.4), translation(20, -70, 13);
  auto samples = observations(anchor, true);
  const double angle = 1.1;
  Eigen::Matrix3d rotation;
  rotation << std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle), 0, 0, 0, 1;
  for (auto& sample : samples) sample.tag = rotation * sample.tag + translation;
  Eigen::Vector2d normal = (rotation * Eigen::Vector3d::UnitY()).head<2>();
  auto seeds = unknown_anchor_seeds(samples, normal, normal.dot(translation.head<2>()), 2.8);
  ASSERT_EQ(seeds.points.size(), 1);
  EXPECT_LT((seeds.points.front() - rotation * anchor - translation).norm(), 1e-8);
}
