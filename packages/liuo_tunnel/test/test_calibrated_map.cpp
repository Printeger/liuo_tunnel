#include <gtest/gtest.h>
#include <unistd.h>
#include <liuo_tunnel/uwb/calibrated_map.hpp>
#include <gtsam/linear/HessianFactor.h>
using namespace liuo_tunnel;
namespace {
nlohmann::json sample() {
  auto j = nlohmann::json{
    {"schema_version", "liuo_joint_map_v1"},
    {"frame", "map"},
    {"anchor_units", "m"},
    {"reference_units", {"mrad", "m"}},
    {"reference_origin_m", {0, 0, 0}},
    {"status", "PROVISIONAL_INITIALIZED"},
    {"unresolved_branches", nlohmann::json::array()},
    {"sensor_identity", {{"test", 1}}},
    {"map_id", "test"},
    {"coordinate_gauge", "local"},
    {"source_run", "test_run"},
    {"approximation", "joint Gaussian"},
    {"source_identity", {{"schema_version", "liuo_map_source_identity_v1"}, {"config_sha256", "fixture"}}},
    {"installation_provenance", {{"already_absorbed", true}}},
    {"not_initialized", nlohmann::json::array()},
    {"estimated_vehicle_bounds_m", {{0, 0, 0}, {40, 1, 2}}},
    {"anchors", {{{"id", 91}, {"mean_m", {0, 2, 3}}}, {{"id", 4}, {"mean_m", {200, -2, 4}}}, {{"id", 8}, {"mean_m", {400, 2, 1}}}}},
    {"reference_mean", {1570.0, .1}}};
  std::vector<std::vector<double>> c(11, std::vector<double>(11, .01));
  for (size_t i = 0; i < 11; ++i) c[i][i] += .1;
  j["joint_covariance"] = c;
  return j;
}
}  // namespace
TEST(CalibratedMap, RoundTripKeepsJointInformationAndCost) {
  const auto m = CalibratedMap::parse(sample(), {{"test", 1}});
  auto directory = std::filesystem::temp_directory_path() / ("joint_map_test_" + std::to_string(getpid()));
  m.save_atomic(directory / "map.json");
  const auto loaded = CalibratedMap::load(directory / "map.json", {{"test", 1}});
  EXPECT_EQ(loaded.ids, m.ids);
  EXPECT_EQ(loaded.document, m.document);
  CalibratedMapPrior prior(loaded);
  gtsam::Values values;
  auto keys = m.keys();
  for (size_t i = 0; i < m.ids.size(); ++i) values.insert(keys[i], gtsam::Point3(m.means[i] + gtsam::Vector3(.1, -.2, .3)));
  values.insert(keys.back(), gtsam::Vector2(m.reference + gtsam::Vector2(.2, -.1)));
  const auto error = prior.unwhitenedError(values);
  EXPECT_NEAR(prior.error(values), .5 * error.dot(m.covariance.ldlt().solve(error)), 1e-10);
  const auto h = gtsam::HessianFactor(*prior.linearize(values)).information();
  EXPECT_LT((h - m.covariance.inverse()).norm(), 1e-9);
  EXPECT_GT(h.topRightCorner(9, 2).norm(), .1);
  EXPECT_FALSE(std::filesystem::exists(directory / "map.json.tmp"));
  std::filesystem::remove_all(directory);
}
TEST(CalibratedMap, RejectsAmbiguityCorruptionAndExtrinsicMismatch) {
  auto j = sample();
  j["unresolved_branches"] = {"mirror"};
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  j = sample();
  j["joint_covariance"][0][0] = -1;
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  EXPECT_THROW(CalibratedMap::parse(sample(), {{"test", 2}}), std::invalid_argument);
  j = sample();
  j["source_identity"]["commands"] = {"simulation", "--sim-start"};
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  j = sample();
  j["anchors"][1]["id"] = 91;
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
}
TEST(CalibratedMap, StaticLedgerIsBoundToTheSameAtomicPosterior) {
  auto document = sample();
  StaticAnchorEdges edges("network", "deployment", 16);
  edges.commit(edges.prepare({{"network", "deployment", 91, 4, "physical-1", "independent-gaussian-v1", 200, .01}}));
  document["static_edge_ledger"] = edges.absorbed_checkpoint();
  EXPECT_THROW(CalibratedMap::parse(document, {{"test", 1}}), std::invalid_argument);
  document["schema_version"] = "liuo_joint_map_static_v2";
  const auto map = CalibratedMap::parse(document, {{"test", 1}});
  auto directory = std::filesystem::temp_directory_path() / ("static_map_test_" + std::to_string(getpid()));
  map.save_atomic(directory / "map.json");
  auto loaded = CalibratedMap::load(directory / "map.json", {{"test", 1}});
  EXPECT_EQ(loaded.document.at("static_edge_ledger"), document.at("static_edge_ledger"));
  EXPECT_TRUE(loaded.covariance.isApprox(map.covariance, 0));
  document["static_edge_ledger"]["leaves"][0]["first"] = 500;
  EXPECT_THROW(CalibratedMap::parse(document, {{"test", 1}}), std::invalid_argument);
  std::filesystem::remove_all(directory);
}

namespace {
nlohmann::json anchor_only_sample() {
  auto j = sample();
  j["schema_version"] = "liuo_anchor_map_v3";
  j["state_layout"] = "A_only";
  j["static_information_policy"] = "AA_DISABLED";
  j["uwb_information"] = {{"use_aa", false}, {"accepted_aa_measurements", 0}};
  j.erase("reference_units");
  j.erase("reference_origin_m");
  j.erase("reference_mean");
  j["anchors"].erase(2);
  std::vector<std::vector<double>> covariance(6, std::vector<double>(6, .02));
  for (size_t i = 0; i < 6; ++i) covariance[i][i] += .1;
  j["joint_covariance"] = covariance;
  return j;
}
}  // namespace
TEST(CalibratedMap, AnchorOnlyRoundTripRetainsCrossInformationWithoutAxisState) {
  const auto map = CalibratedMap::parse(anchor_only_sample(), {{"test", 1}});
  EXPECT_FALSE(map.has_reference);
  ASSERT_EQ(map.keys().size(), 2);
  EXPECT_EQ(map.mean_vector().size(), 6);
  for (auto key : map.keys()) EXPECT_EQ(gtsam::Symbol(key).chr(), 'a');
  auto directory = std::filesystem::temp_directory_path() / ("anchor_map_test_" + std::to_string(getpid()));
  map.save_atomic(directory / "map.json");
  const auto loaded = CalibratedMap::load(directory / "map.json", {{"test", 1}});
  EXPECT_EQ(loaded.document, map.document);
  CalibratedMapPrior prior(loaded);
  gtsam::Values values;
  for (size_t i = 0; i < 2; ++i) values.insert(map.keys()[i], gtsam::Point3(map.means[i] + gtsam::Point3(.1, -.2, .3)));
  const auto error = prior.unwhitenedError(values);
  EXPECT_NEAR(prior.error(values), .5 * error.dot(map.covariance.ldlt().solve(error)), 1e-10);
  const auto information = gtsam::HessianFactor(*prior.linearize(values)).information();
  EXPECT_LT((information - map.covariance.inverse()).norm(), 1e-9);
  EXPECT_GT(information.topRightCorner(3, 3).norm(), .1);
  std::filesystem::remove_all(directory);
}
TEST(CalibratedMap, AnchorOnlyRejectsHiddenAxisAndUnaccountedAA) {
  auto j = anchor_only_sample();
  j["reference_mean"] = {0, 0};
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  j = anchor_only_sample();
  j["uwb_information"]["accepted_aa_measurements"] = 1;
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  j = anchor_only_sample();
  j["static_information_policy"] = "EXACT_LEDGER";
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
  j = anchor_only_sample();
  j["joint_covariance"][0].push_back(0);
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
}
TEST(CalibratedMap, AnchorOnlyLedgerPreservesAbsorbedPhysicalIdentity) {
  auto j = anchor_only_sample();
  j["static_information_policy"] = "EXACT_LEDGER";
  StaticAnchorEdges edges("network", "deployment", 16);
  edges.commit(edges.prepare({{"network", "deployment", 91, 4, "physical-1", "independent-gaussian-v1", 200, .01}}));
  j["static_edge_ledger"] = edges.absorbed_checkpoint();
  const auto map = CalibratedMap::parse(j, {{"test", 1}});
  StaticAnchorEdges loaded("network", "deployment", 16, map.document.at("static_edge_ledger"));
  EXPECT_EQ(loaded.absorbed_checkpoint(), edges.absorbed_checkpoint());
  j["static_edge_ledger"]["leaves"][0]["first"] = 500;
  EXPECT_THROW(CalibratedMap::parse(j, {{"test", 1}}), std::invalid_argument);
}
