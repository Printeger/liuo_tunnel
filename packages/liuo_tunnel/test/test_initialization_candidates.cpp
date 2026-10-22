#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
using namespace liuo_tunnel;
namespace {
std::array<RangeSeedSphere, 3> spheres(const gtsam::Point3& p) {
  std::array<RangeSeedSphere, 3> s{{{{0, 0, 0}, 0}, {{400, 3, 0}, 0}, {{800, -2, 4}, 0}}};
  for (auto& a : s) a.radius = (a.anchor - p).norm();
  return s;
}
}  // namespace
TEST(InitializationCandidates, KeepsBothMirrorsAndLeverForEveryYaw) {
  const gtsam::Point3 p(180, .8, 2.4);
  const auto input = spheres(p);
  const auto result = intersect_range_seeds(input);
  ASSERT_EQ(result.reason, "MIRROR_SEED_PAIR");
  ASSERT_EQ(result.tags.size(), 2);
  EXPECT_LT(std::min((result.tags[0] - p).norm(), (result.tags[1] - p).norm()), 1e-8);
  EXPECT_GT((result.tags[0] - result.tags[1]).norm(), 1);
  for (const auto& tag : result.tags)
    for (const auto& s : input) EXPECT_NEAR((tag - s.anchor).norm(), s.radius, 1e-9);
  const gtsam::Vector3 lever(.1, .15, .25);
  const auto gravity = gtsam::Rot3::RzRyRx(.08, -.12, 0);
  const auto poses = pose_seeds_from_tags(result.tags, gravity, lever);
  ASSERT_EQ(poses.size(), 16);
  for (size_t k = 0; k < poses.size(); ++k) {
    EXPECT_LT((poses[k].transformFrom(lever) - result.tags[k / 8]).norm(), 1e-10);
    EXPECT_LT((poses[k].rotation().unrotate(gtsam::Vector3::UnitZ()) - gravity.unrotate(gtsam::Vector3::UnitZ())).norm(), 1e-10);
  }
}
TEST(InitializationCandidates, DoesNotDependOnWorldOriginOrAnchorOrder) {
  auto input = spheres({180, .8, 2.4});
  const auto baseline = intersect_range_seeds(input);
  const gtsam::Pose3 transform(gtsam::Rot3::RzRyRx(.2, -.3, 1.1), {120, -60, 80});
  for (auto& s : input) s.anchor = transform.transformFrom(s.anchor);
  std::swap(input[0], input[2]);
  const auto changed = intersect_range_seeds(input);
  ASSERT_EQ(changed.tags.size(), 2);
  for (const auto& tag : baseline.tags) {
    const auto expected = transform.transformFrom(tag);
    EXPECT_LT(std::min((expected - changed.tags[0]).norm(), (expected - changed.tags[1]).norm()), 1e-8);
  }
}
TEST(InitializationCandidates, RejectsInconsistentAndDegenerateInput) {
  auto input = spheres({180, .8, 2.4});
  for (auto& s : input) s.radius = 1;
  EXPECT_EQ(intersect_range_seeds(input).reason, "SEED_SPHERES_DO_NOT_INTERSECT");
  input[2].anchor = 2 * input[1].anchor;
  EXPECT_EQ(intersect_range_seeds(input).reason, "DEGENERATE_SEED_ANCHORS");
  input[0].radius = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(intersect_range_seeds(input).reason, "INVALID_SEED_SPHERE");
  EXPECT_THROW(pose_seeds_from_tags({}, gtsam::Rot3(), gtsam::Vector3::Zero(), 1000), std::invalid_argument);
}
TEST(InitializationCandidates, TangentIsNotAnArtificialMirrorPair) {
  std::array<RangeSeedSphere, 3> input{{{{0, 0, 0}, 5}, {{6, 0, 0}, 5}, {{0, 8, 0}, 5}}};
  const auto result = intersect_range_seeds(input);
  ASSERT_EQ(result.tags.size(), 1);
  EXPECT_EQ(result.reason, "TANGENT_SEED_SPHERES");
  EXPECT_LT((result.tags[0] - gtsam::Point3(3, 4, 0)).norm(), 1e-10);
}
