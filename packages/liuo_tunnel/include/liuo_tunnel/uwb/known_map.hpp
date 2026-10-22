#pragma once
#include <liuo_tunnel/uwb/range_factors.hpp>
#include <liuo_tunnel/common/sensors.hpp>
#include <Eigen/Cholesky>
#include <set>
namespace liuo_tunnel {
struct KnownMap {
  std::string id, mode, frame, provenance;
  nlohmann::json installation;
  std::vector<uint32_t> ids;
  std::vector<gtsam::Point3> means;
  gtsam::Matrix covariance;
  size_t index(uint32_t id) const {
    auto it = std::find(ids.begin(), ids.end(), id);
    if (it == ids.end()) throw std::invalid_argument("UNKNOWN_ANCHOR");
    return std::distance(ids.begin(), it);
  }
  bool surveyed() const { return mode == "surveyed_map"; }
  static KnownMap parse(const nlohmann::json& j, const gtsam::Point3& lever) {
    KnownMap m;
    if (j.at("schema_version") != "liuo_known_map_v1" || j.at("units") != "m" || j.at("axis_convention") != "ENU" || j.at("valid") != true)
      throw std::invalid_argument("MAP_INVALID: schema/units/axis/validity");
    m.installation = j.value("installation", nlohmann::json());
    m.id = j.at("map_id");
    m.mode = j.at("mode");
    m.frame = j.at("frame");
    m.provenance = j.at("provenance");
    if (m.id.empty() || m.frame != "map" || m.provenance.empty() || (m.mode != "oracle_map" && m.mode != "surveyed_map"))
      throw std::invalid_argument("MAP_INVALID: identity/frame/mode");
    if ((vector3(j.at("tag_lever_m")) - lever).norm() > 1e-12) throw std::invalid_argument("MAP_EXTRINSIC_CONFLICT");
    std::set<uint32_t> seen;
    for (const auto& a : j.at("anchors")) {
      uint32_t id = a.at("id");
      if (!seen.insert(id).second) throw std::invalid_argument("MAP_DUPLICATE_ID");
      m.ids.push_back(id);
      m.means.push_back(vector3(a.at("mean_m")));
    }
    if (m.ids.size() < 4 || m.ids.size() > 1000) throw std::invalid_argument("MAP_INVALID: anchor count");
    if (m.surveyed()) {
      const auto& c = j.at("joint_covariance_m2");
      const size_t n = 3 * m.ids.size();
      if (c.size() != n) throw std::invalid_argument("MAP_COVARIANCE_DIMENSION");
      m.covariance.resize(n, n);
      for (size_t r = 0; r < n; r++) {
        if (c[r].size() != n) throw std::invalid_argument("MAP_COVARIANCE_DIMENSION");
        for (size_t k = 0; k < n; k++) m.covariance(r, k) = c[r][k].get<double>();
      }
      if (!m.covariance.allFinite() || !m.covariance.isApprox(m.covariance.transpose(), 1e-12) || Eigen::LLT<gtsam::Matrix>(m.covariance).info() != Eigen::Success)
        throw std::invalid_argument("MAP_COVARIANCE_NOT_SPD");
    } else if (j.contains("joint_covariance_m2"))
      throw std::invalid_argument("ORACLE_CANNOT_HIDE_MAP_UNCERTAINTY");
    return m;
  }
  static KnownMap load(const std::string& path, const gtsam::Point3& lever) {
    std::ifstream stream(path);
    if (!stream) throw std::invalid_argument("MAP_MISSING: " + path);
    return parse(nlohmann::json::parse(stream), lever);
  }
};
}  // namespace liuo_tunnel
