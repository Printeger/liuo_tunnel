#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <liuo_tunnel_msgs/msg/uwb_range.hpp>
#include <liuo_tunnel/common/sensors.hpp>
#include <liuo_tunnel/util/run_log_manager.hpp>
#include <liuo_tunnel/util/logging.hpp>
#include <liuo_tunnel_sim/rail_model.hpp>
#include <liuo_tunnel_sim/rail_world.hpp>
#include <liuo_tunnel_sim/first_hit_lidar_renderer.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
using liuo_tunnel::RunLogManager;
namespace liuo_tunnel_sim {
class RailSim : public rclcpp::Node {
 public:
 RailSim():Node("liuo_rail_sim"), renderer(make_renderer()) {
  cfg=liuo_tunnel::load_config(declare_parameter<std::string>("config_file",""));
  seed=declare_parameter<int>("seed",0); duration=declare_parameter<double>("duration_s",45);
  reset_at=declare_parameter<double>("reset_at_s",-1);
  T_BL=liuo_tunnel::lidar_extrinsic(cfg); tag=liuo_tunnel::vector3(cfg.at("sensors").at("T_BU_translation"));
  logger=liuo_tunnel::create_module_logger("simulation");
  auto& logs=RunLogManager::instance(); logs.write_run_info({{"scan_model","snapshot"},{"seed",std::to_string(seed)}});
  truth.open(logs.export_path("simulation/truth.csv")); truth<<std::setprecision(16)<<"session,stamp,x,y,z,vx,vy,vz\n";
  uwblog.open(logs.export_path("simulation/uwb.csv")); uwblog<<std::setprecision(16)<<"session,sequence,stamp,a_type,a,b,true_range,range,variance\n";
  timing.open(logs.profiling_path("simulation.csv")); timing<<"stamp,render_ms,dda_visits,points,world_points\n";
  clock_pub=create_publisher<rosgraph_msgs::msg::Clock>("/clock",10);
  imu_pub=create_publisher<sensor_msgs::msg::Imu>("/sensors/imu",rclcpp::SensorDataQoS().keep_last(2048));
  cloud_pub=create_publisher<sensor_msgs::msg::PointCloud2>("/sensors/lidar/points",rclcpp::SensorDataQoS().keep_last(32));
  truth_pub=create_publisher<nav_msgs::msg::Odometry>("/sim/truth/odom",100);
  uwb_pub=create_publisher<liuo_tunnel_msgs::msg::UwbRange>("/sensors/uwb/range",rclcpp::SensorDataQoS().keep_last(2048));
  for(int i=0;i<13;i++) links.push_back({0,0,i,0,0});
  for(int i=0;i<13;i++) for(int j=i+1;j<13;j++) if(reachable(anchor(i),anchor(j))) links.push_back({1,i,j,0,0});
  timer=create_wall_timer(std::chrono::milliseconds(5),[this]{tick();});
  logger->info("snapshot simulation ready seed={} duration={}s",seed,duration);
 }
 private:
 static FirstHitLidarConfig make_renderer() { FirstHitLidarConfig c; c.max_range_m=40; c.vertical_min_deg=-25; c.vertical_max_deg=25; return c; }
 builtin_interfaces::msg::Time stamp(double t) { return rclcpp::Time(static_cast<int64_t>(std::llround(t*1e9))); }
 void world(double x) {
  int block=static_cast<int>(std::floor(x/20)); if(block==loaded) return; loaded=block;
  // Bounded local block window. UWB uses the complete analytic convex world, never this LiDAR cache.
  auto p=rail_world(block);
  world_points=p.size(); std::string why; if(!renderer.loadWorld(p,&why)) throw std::runtime_error("renderer world: "+why);
 }
 void tick() {
  double elapsed=total_ticks*.005;
  if(elapsed>=duration) { logger->info("simulation finished ticks={} sessions={}",total_ticks,session); rclcpp::shutdown(); return; }
  if(reset_at>0 && !reset_done && elapsed>=reset_at) {reset_done=true; ticks=0; session++; sequence=0; exchange=0; ba.setZero(); bg.setZero(); for(auto& l:links){l.next=0;l.samples=0;} logger->warn("explicit simulation clock reset session={}",session);}
  double t=ticks*.005, ros_t=1+t; auto m=motion(t);
  rosgraph_msgs::msg::Clock clock; clock.clock=stamp(ros_t); clock_pub->publish(clock);
  sensor_msgs::msg::Imu imu; imu.header.stamp=stamp(ros_t); imu.header.frame_id="imu"; imu.orientation_covariance[0]=-1;
  auto f=specific_force(m); auto w=m.omega; auto s=cfg.at("sensors");
  for(int j=0;j<3;j++) {
   ba[j]+=s.at("sigma_ba_density").get<double>()*std::sqrt(.005)*normal(seed,20,ticks,j);
   bg[j]+=s.at("sigma_bg_density").get<double>()*std::sqrt(.005)*normal(seed,21,ticks,j);
   f[j]+=ba[j]+s.at("sigma_acc_sample").get<double>()*normal(seed,10,ticks,j);
   w[j]+=bg[j]+s.at("sigma_gyro_sample").get<double>()*normal(seed,11,ticks,j);
   imu.linear_acceleration_covariance[j*3+j]=std::pow(s.at("sigma_acc_sample").get<double>(),2);
   imu.angular_velocity_covariance[j*3+j]=std::pow(s.at("sigma_gyro_sample").get<double>(),2);
  }
  imu.linear_acceleration.x=f.x(); imu.linear_acceleration.y=f.y(); imu.linear_acceleration.z=f.z();
  imu.angular_velocity.x=w.x(); imu.angular_velocity.y=w.y(); imu.angular_velocity.z=w.z(); imu_pub->publish(imu);
  while(exchange*.004<=t+1e-10) { double event=exchange++*.004; schedule(event); }
  if(ticks%20==0) {
   world(m.p.x()); Eigen::Isometry3d pose=Eigen::Isometry3d::Identity(); pose.linear()=m.R; pose.translation()=m.p;
   auto scan=renderer.render(pose*T_BL);
   sensor_msgs::msg::PointCloud2 cloud; cloud.header.stamp=stamp(ros_t); cloud.header.frame_id="lidar";
   sensor_msgs::PointCloud2Modifier mod(cloud); mod.setPointCloud2Fields(5,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,"intensity",1,sensor_msgs::msg::PointField::FLOAT32,"time",1,sensor_msgs::msg::PointField::FLOAT32); mod.resize(scan.hits.size());
   sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z"),pi(cloud,"intensity"),pt(cloud,"time");
   size_t index=0; for(const auto& hit:scan.hits) {
    Eigen::Vector3d q=(pose*T_BL).inverse()*Eigen::Vector3d(hit.x,hit.y,hit.z); q*=1+s.at("sigma_lidar_range").get<double>()*normal(seed,30,ticks/20,index++)/q.norm();
    *px=q.x();*py=q.y();*pz=q.z();*pi=1;*pt=0; ++px;++py;++pz;++pi;++pt;
   }
   cloud_pub->publish(cloud);
   nav_msgs::msg::Odometry odom; odom.header.stamp=stamp(ros_t); odom.header.frame_id="world"; odom.child_frame_id="imu";
   odom.pose.pose.position.x=m.p.x(); odom.pose.pose.position.y=m.p.y(); odom.pose.pose.position.z=m.p.z(); odom.pose.pose.orientation.w=1; odom.twist.twist.linear.x=m.v.x(); truth_pub->publish(odom);
   truth<<session<<','<<ros_t<<','<<m.p.x()<<','<<m.p.y()<<','<<m.p.z()<<','<<m.v.x()<<','<<m.v.y()<<','<<m.v.z()<<'\n';
   timing<<ros_t<<','<<scan.stats.render_latency_ms<<','<<scan.stats.dda_voxel_visits<<','<<scan.hits.size()<<','<<world_points<<'\n';
  }
  ticks++;total_ticks++;
 }
 struct Link {int type,a,b; double next; uint64_t samples;};
 void schedule(double t) {
  auto m=motion(t); Eigen::Vector3d tag_w=m.p+m.R*tag;
  // TA priority; AA round-robin by oldest deadline; one exchange per 4ms slot.
  int best=-1; double deadline=1e100;
  for(size_t i=0;i<links.size();i++) {auto& l=links[i]; if(l.next>t+1e-9) continue;
   Eigen::Vector3d a=l.type?anchor(l.a):tag_w;
   if(!reachable(a,anchor(l.b))) {l.next=t+(l.type?1:.05); continue;}
   double rank=l.next+(l.type?10000:0); if(rank<deadline){deadline=rank;best=i;}
  }
  if(best<0)return; auto& l=links[best]; Eigen::Vector3d a=l.type?anchor(l.a):tag_w,b=anchor(l.b);
  // Straight rectangular tunnel is convex: all scheduled endpoints are interior LOS.
  liuo_tunnel_msgs::msg::UwbRange u; u.header.stamp=stamp(1+t); u.producer_id=1;u.session_id=session;u.sequence=sequence++;
  u.node_a_type=l.type;u.node_a_id=l.a;u.node_b_type=1;u.node_b_id=l.b;u.status=0;u.variance_m2=.01;u.propagation_status=0;
  u.range_m=range(a,b,seed,1000+l.type*10000+l.a*100+l.b,l.samples++);uwb_pub->publish(u);
  uwblog<<session<<','<<u.sequence<<','<<1+t<<','<<l.type<<','<<l.a<<','<<l.b<<','<<(a-b).norm()<<','<<u.range_m<<",0.01\n";
  l.next=t+(l.type?1:.05);
 }
 nlohmann::json cfg; int seed=0,loaded=-100; double duration=45,reset_at=-1; bool reset_done=false;
 uint64_t ticks=0,total_ticks=0,exchange=0,session=1,sequence=0; size_t world_points=0;
 Eigen::Vector3d ba=Eigen::Vector3d::Zero(),bg=Eigen::Vector3d::Zero(),tag; Eigen::Isometry3d T_BL;
 FirstHitLidarRenderer renderer; std::vector<Link> links; std::ofstream truth,uwblog,timing; std::shared_ptr<spdlog::logger> logger;
 rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
 rclcpp::Publisher<liuo_tunnel_msgs::msg::UwbRange>::SharedPtr uwb_pub;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub; rclcpp::TimerBase::SharedPtr timer;
};
}
int main(int argc,char** argv) { rclcpp::init(argc,argv); try {RunLogManager::initialize("simulation"); rclcpp::spin(std::make_shared<liuo_tunnel_sim::RailSim>()); return 0;} catch(const std::exception& e){spdlog::error("simulation failed: {}",e.what());rclcpp::shutdown();return 1;} }
