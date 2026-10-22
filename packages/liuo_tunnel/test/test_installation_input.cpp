#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/installation_input.hpp>
using namespace liuo_tunnel;
namespace {
nlohmann::json sample() {
  return {{"schema_version", "liuo_lateral_installation_v1"}, {"units", "m"}, {"reference_model", "shared_straight_horizontal_axis"},
          {"provenance", "test measurement"}, {"reference_id", "axis"}, {"positive_side_convention", "fixed signed side"}, {"scope", "test"},
          {"records", {{{"anchor_id", 91}, {"signed_offset_m", 2.8}}, {{"anchor_id", 4}, {"signed_offset_m", -2.7}}}},
          {"joint_covariance_m2", {{.003, .0004}, {.0004, .003}}}};
}
}
TEST(InstallationInput, PreservesCorrelationsWithoutSortingIds) {
  auto j = sample();
  auto input = InstallationInput::parse(j);
  EXPECT_EQ(input.ids, (std::vector<uint32_t>{91, 4}));
  EXPECT_DOUBLE_EQ(input.covariance(0, 1), .0004);
  std::swap(j["records"][0], j["records"][1]);
  auto reversed = InstallationInput::parse(j);
  EXPECT_EQ(reversed.ids, (std::vector<uint32_t>{4, 91}));
  EXPECT_DOUBLE_EQ(reversed.offsets[0], -2.7);
}
TEST(InstallationInput, RejectsCoordinatesAndUnapprovedPriors) {
  for (auto key : {"anchors", "mean_m", "height_prior", "order", "spacing_m", "coordinate_reference"}) {
    auto j = sample();
    j[key] = 400;
    EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
    j = sample();
    j["records"][0][key] = 400;
    EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
  }
}
TEST(InstallationInput, RejectsInvalidIdentityAndCovariance) {
  auto j = sample();
  j["records"][1]["anchor_id"] = 91;
  EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
  j = sample();
  j["records"][0]["anchor_id"] = -1;
  EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
  j = sample();
  j["joint_covariance_m2"] = {{1, 1}, {1, 1}};
  EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
  j = sample();
  j["joint_covariance_m2"][0][1] = .001;
  EXPECT_THROW(InstallationInput::parse(j), std::invalid_argument);
}
