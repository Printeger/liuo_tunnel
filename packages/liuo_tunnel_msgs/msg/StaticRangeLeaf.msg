# One independent physical exchange. Identity survives retransmission and
# driver restart; never substitute a delivery frame or a coordinate posterior.
string<=256 physical_id
uint32 physical_source
builtin_interfaces/Time acquisition_stamp
bool acquisition_stamp_valid
string<=64 time_basis
float64 range_m
float64 variance_m2
