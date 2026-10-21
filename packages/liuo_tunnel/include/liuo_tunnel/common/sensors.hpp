#pragma once
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Geometry>
#include <stdexcept>
namespace liuo_tunnel {
inline nlohmann::json load_config(const std::string& path) {
 std::ifstream f(path); if(!f) throw std::runtime_error("missing config_file: "+path);
 auto j=nlohmann::json::parse(f);
 if(j.at("scan_model")!="snapshot") throw std::runtime_error("scan_model: only explicit snapshot is validated in phase one");
 if(j.at("uwb").at("mode")!="record_only") throw std::runtime_error("uwb.mode must be record_only in phase one");
 return j;
}
inline Eigen::Vector3d vector3(const nlohmann::json& j) {
 if(!j.is_array() || j.size()!=3) throw std::runtime_error("extrinsic must contain three components");
 Eigen::Vector3d v(j.at(0).get<double>(),j.at(1).get<double>(),j.at(2).get<double>());
 if(!v.allFinite()) throw std::runtime_error("nonfinite extrinsic"); return v;
}
inline Eigen::Isometry3d lidar_extrinsic(const nlohmann::json& j) {
 Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
 t.translation()=vector3(j.at("sensors").at("T_BL_translation"));
 double yaw=j.at("sensors").at("T_BL_yaw"); if(!std::isfinite(yaw)) throw std::runtime_error("T_BL_yaw invalid");
 t.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix(); return t;
}
}
