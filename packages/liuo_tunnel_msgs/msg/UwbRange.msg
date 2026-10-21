# header.stamp: 距离所对应的有效测量时刻，已转换到统一 ROS 时间轴
# header.frame_id: 空字符串；标量距离不属于某个向量表达坐标系
std_msgs/Header header

# 一个发布源启动会话中的序号；唯一身份为 producer/session/sequence
uint32 producer_id
uint64 session_id
uint64 sequence

uint8 NODE_TAG=0
uint8 NODE_ANCHOR=1
uint8 node_a_type
uint32 node_a_id
uint8 node_b_type
uint32 node_b_id

uint8 STATUS_OK=0
uint8 STATUS_TIMEOUT=1
uint8 STATUS_INVALID=2
uint8 status

float64 range_m
float64 variance_m2

uint8 LOS_UNKNOWN=0
uint8 LOS_REPORTED=1
uint8 NLOS_REPORTED=2
uint8 propagation_status
