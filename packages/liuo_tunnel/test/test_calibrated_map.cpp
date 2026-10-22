#include <gtest/gtest.h>
#include <unistd.h>
#include <liuo_tunnel/uwb/calibrated_map.hpp>
#include <gtsam/linear/HessianFactor.h>
using namespace liuo_tunnel;
namespace {
nlohmann::json sample() {
  auto j = nlohmann::json{{"schema_version", "liuo_joint_map_v1"}, {"frame", "map"}, {"anchor_units", "m"},
    {"reference_units", {"mrad", "m"}}, {"reference_origin_m", {0,0,0}}, {"status", "PROVISIONAL_INITIALIZED"},
    {"unresolved_branches", nlohmann::json::array()}, {"sensor_identity", {{"test", 1}}}, {"map_id", "test"}, {"coordinate_gauge", "local"},
    {"source_run", "test_run"}, {"approximation", "joint Gaussian"}, {"source_identity", {{"hash", "fixture"}}},
    {"installation_provenance", {{"already_absorbed", true}}}, {"not_initialized", nlohmann::json::array()}, {"estimated_vehicle_bounds_m", {{0,0,0},{40,1,2}}},
    {"anchors", {{{"id", 91}, {"mean_m", {0,2,3}}}, {{"id", 4}, {"mean_m", {200,-2,4}}}, {{"id", 8}, {"mean_m", {400,2,1}}}}},
    {"reference_mean", {1570.0, .1}}};
  std::vector<std::vector<double>> c(11, std::vector<double>(11, .01));
  for (size_t i=0;i<11;++i) c[i][i]+=.1;
  j["joint_covariance"]=c;
  return j;
}
}
TEST(CalibratedMap, RoundTripKeepsJointInformationAndCost) {
  const auto m=CalibratedMap::parse(sample(), {{"test",1}});
  auto directory=std::filesystem::path("/home/dev/code/ws_tunnel/src/liuo_tunnel/build/liuo_tunnel")/("joint_map_test_"+std::to_string(getpid()));
  m.save_atomic(directory/"map.json");
  const auto loaded=CalibratedMap::load(directory/"map.json", {{"test",1}});
  EXPECT_EQ(loaded.ids,m.ids);
  EXPECT_EQ(loaded.document,m.document);
  CalibratedMapPrior prior(loaded);
  gtsam::Values values;
  auto keys=m.keys();
  for(size_t i=0;i<m.ids.size();++i) values.insert(keys[i], gtsam::Point3(m.means[i]+gtsam::Vector3(.1,-.2,.3)));
  values.insert(keys.back(),gtsam::Vector2(m.reference+gtsam::Vector2(.2,-.1)));
  const auto error=prior.unwhitenedError(values);
  EXPECT_NEAR(prior.error(values),.5*error.dot(m.covariance.ldlt().solve(error)),1e-10);
  const auto h=gtsam::HessianFactor(*prior.linearize(values)).information();
  EXPECT_LT((h-m.covariance.inverse()).norm(),1e-9);
  EXPECT_GT(h.topRightCorner(9,2).norm(),.1);
  EXPECT_FALSE(std::filesystem::exists(directory/"map.json.tmp"));
  std::filesystem::remove_all(directory);
}
TEST(CalibratedMap, RejectsAmbiguityCorruptionAndExtrinsicMismatch) {
  auto j=sample(); j["unresolved_branches"]={"mirror"};
  EXPECT_THROW(CalibratedMap::parse(j,{{"test",1}}),std::invalid_argument);
  j=sample();j["joint_covariance"][0][0]=-1;
  EXPECT_THROW(CalibratedMap::parse(j,{{"test",1}}),std::invalid_argument);
  EXPECT_THROW(CalibratedMap::parse(sample(),{{"test",2}}),std::invalid_argument);
  j=sample();j["anchors"][1]["id"]=91;
  EXPECT_THROW(CalibratedMap::parse(j,{{"test",1}}),std::invalid_argument);
}
