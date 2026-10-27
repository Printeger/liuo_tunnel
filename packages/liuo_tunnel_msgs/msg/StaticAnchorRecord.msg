# Project-local cached static AA channel, independent of the raw shared batch.
# Source evidence is configured out of band and must match source_identity.
# Header stamp is software publication time; receiver records its own time.
std_msgs/Header header
string<=256 network
string<=256 deployment_epoch
string<=256 source_identity
string<=256 record_id
uint64 revision
uint32 first
uint32 second
string<=64 error_model
bool valid
# Exact physical coverage. Each ID has one immutable leaf below. A later
# revision does not itself add information; only previously unseen leaves do.
string<=256[<=128] covered_blocks
StaticRangeLeaf[<=128] leaves
