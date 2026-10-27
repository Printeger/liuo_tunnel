#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/static_record_input.hpp>

using namespace liuo_tunnel;

TEST(StaticRecordInput, OldAcquisitionIsValidWithoutVehicleTimeBracket) {
  const nlohmann::json config = {
    {"network", "network"},
    {"deployment_epoch", "deployment"},
    {"source_identity", "frozen-delivered-data"},
    {"noise_evidence", "synthetic independent range noise"},
    {"independence_evidence", "one physical sequence per exchange"},
    {"time_basis", "shared_simulation_clock"},
    {"variance_m2", 0.01}};
  StaticRecordInput input(config);
  liuo_tunnel_msgs::msg::StaticAnchorRecord record;
  record.network = "network";
  record.deployment_epoch = "deployment";
  record.source_identity = "frozen-delivered-data";
  record.record_id = "edge-record";
  record.revision = 1;
  record.first = 4;
  record.second = 9;
  record.error_model = "independent-gaussian-v1";
  record.valid = true;
  record.header.stamp.sec = 10000;
  liuo_tunnel_msgs::msg::StaticRangeLeaf leaf;
  leaf.physical_id = "physical-exchange-1";
  leaf.physical_source = 9;
  leaf.acquisition_stamp.sec = 1;
  leaf.acquisition_stamp_valid = true;
  leaf.time_basis = "shared_simulation_clock";
  leaf.range_m = 10;
  leaf.variance_m2 = 0.01;
  record.leaves.push_back(leaf);
  record.covered_blocks.push_back(leaf.physical_id);
  auto accepted = input.validate(record);
  ASSERT_EQ(accepted.size(), 1u);
  EXPECT_EQ(accepted[0].physical_id, "physical-exchange-1");
  record.covered_blocks[0] = "different-exchange";
  EXPECT_THROW(input.validate(record), std::invalid_argument);
  record.covered_blocks[0] = leaf.physical_id;
  record.source_identity = "unconfigured-source";
  EXPECT_THROW(input.validate(record), std::invalid_argument);
}
