#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/observation_input.hpp>
using namespace liuo_tunnel;
nlohmann::json input_config() {
  return {{"network_instance","test"},{"receiver_id",0},
    {"nodes", {{{"logical_id",0},{"type",0},{"internal_id",0}},{{"logical_id",1},{"type",1},{"internal_id",0}},{{"logical_id",2},{"type",1},{"internal_id",1}}}},
    {"model",{{"id","test"},{"source","synthetic"},{"clock_domain","test"},{"time_evidence","exact"},{"noise_evidence","assumption"},
      {"independence_evidence","generated samples"},{"ta_sigma_m",.1},{"aa_sigma_m",.1},{"max_time_uncertainty_s",.01}}}};
}
ObservationInput::Observation observation(uint32_t seq=1) {
  ObservationInput::Observation o;
  o.node_b=1; o.source_session_id=uint64_t(1)<<40; o.source_frame_seq=seq; o.source_frame_seq_valid=true; o.shard_count=1;
  o.time_basis=1; o.source_frame_stamp.sec=1; o.distance_m=10; o.time_uncertainty_s=0; o.pair_measurement_freshness_verified=true;
  o.fp_rssi=o.rx_rssi=NAN; o.source_hold_age_ms=65535;
  return o;
}
TEST(ObservationInput, PeersRetransmitAndWrap) {
  ObservationInput input(input_config()); ObservationInput::Batch b; b.driver_session_id=uint64_t(1)<<48;
  RangeObservation out; auto o=observation(0xfffffffeu);
  EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT"); EXPECT_EQ(out.b,0u); EXPECT_EQ(out.a,0u); EXPECT_NE(out.a_type,out.b_type);
  o.node_b=2; EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  o.tx_cycle_seq=100; EXPECT_EQ(input.adapt(b,o,2,out),"DUPLICATE");
  o.source_frame_seq=0; EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  o.source_frame_seq=0xffffffffu; EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  o.source_frame_seq=200; EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  o.source_frame_seq=0; EXPECT_EQ(input.adapt(b,o,2,out),"IDENTITY_TOO_OLD");
}
TEST(ObservationInput, UnknownEvidenceAndInvalidFields) {
  ObservationInput::Batch b; b.driver_session_id=1; RangeObservation out;
  auto check=[&](auto o){ ObservationInput input(input_config()); return input.adapt(b,o,2,out); };
  auto o=observation(); o.time_basis=0; EXPECT_EQ(check(o),"MEASUREMENT_TIME_UNKNOWN");
  o=observation(); o.time_uncertainty_s=NAN; EXPECT_EQ(check(o),"TIME_UNCERTAINTY_UNSUPPORTED");
  o=observation(); o.pair_measurement_freshness_verified=false; EXPECT_EQ(check(o),"INDEPENDENCE_UNVERIFIED");
  o=observation(); o.source_frame_stamp.nanosec=1000000000; EXPECT_EQ(check(o),"INVALID_SOURCE_TIME");
  o=observation(); o.distance_m=INFINITY; EXPECT_EQ(check(o),"INVALID_DISTANCE");
  auto cfg=input_config(); cfg.erase("model"); ObservationInput input(cfg);
  EXPECT_EQ(input.adapt(b,observation(),2,out),"MODEL_UNAVAILABLE");
  cfg=input_config(); cfg["model"]["ta_sigma_m"]=0; EXPECT_THROW(ObservationInput bad(cfg),std::invalid_argument);
}
TEST(ObservationInput, RetirementAndAAFirstAvailableSource) {
  ObservationInput input(input_config()); ObservationInput::Batch b; b.driver_session_id=1; b.trigger_type=1;
  auto o=observation(); o.kind=1; o.node_a=o.reporter_id=2; o.node_b=1; RangeObservation out;
  EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  o.node_a=o.reporter_id=1; o.node_b=2; EXPECT_EQ(input.adapt(b,o,2,out),"AA_OTHER_SOURCE");
  o.source_session_id++; EXPECT_EQ(input.adapt(b,o,2,out),"AA_OTHER_SOURCE");
  o.source_session_id--; EXPECT_EQ(input.adapt(b,o,2,out),"RETIRED_SOURCE_SESSION");
  b.driver_session_id=2; EXPECT_EQ(input.adapt(b,o,2,out),"ACCEPT_INPUT");
  b.driver_session_id=1; EXPECT_EQ(input.adapt(b,o,2,out),"RETIRED_DRIVER_SESSION");
  input.reset(); b.driver_session_id=2; EXPECT_EQ(input.adapt(b,o,2,out),"RETIRED_DRIVER_SESSION");
}
