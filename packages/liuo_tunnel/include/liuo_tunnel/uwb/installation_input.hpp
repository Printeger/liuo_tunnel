#pragma once
#include <Eigen/Cholesky>
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace liuo_tunnel {
// First-run input deliberately has no coordinate, order or height field.
// Record order is only the row order of the correlated measurement covariance.
struct InstallationInput {
  nlohmann::json source;
  std::vector<uint32_t> ids;
  Eigen::VectorXd offsets;
  Eigen::MatrixXd covariance;

  static InstallationInput parse(const nlohmann::json& j) {
    const std::set<std::string> allowed{
        "schema_version", "provenance", "reference_model", "reference_id", "positive_side_convention", "scope", "units", "seed",
        "independent_from_map_survey", "common_sigma_m", "independent_sigma_m", "records", "joint_covariance_m2"};
    if (!j.is_object()) throw std::invalid_argument("INSTALLATION_OBJECT_REQUIRED");
    for (const auto& item : j.items())
      if (!allowed.count(item.key())) throw std::invalid_argument("INSTALLATION_FORBIDDEN_FIELD: " + item.key());
    if (j.at("schema_version") != "liuo_lateral_installation_v1" || j.at("units") != "m" ||
        j.at("reference_model") != "shared_straight_horizontal_axis")
      throw std::invalid_argument("INSTALLATION_SCHEMA");
    for (const auto* key : {"provenance", "reference_id", "positive_side_convention", "scope"})
      if (j.at(key).get<std::string>().empty()) throw std::invalid_argument("INSTALLATION_MISSING_PROVENANCE");
    const auto& records = j.at("records");
    const auto n = records.size();
    if (!records.is_array() || n == 0 || n > 1000) throw std::invalid_argument("INSTALLATION_COUNT");
    InstallationInput input;
    input.source = j;
    input.offsets.resize(n);
    std::set<uint32_t> seen;
    for (size_t i = 0; i < n; ++i) {
      const auto& r = records[i];
      if (!r.is_object() || r.size() != 2 || !r.contains("anchor_id") || !r.contains("signed_offset_m"))
        throw std::invalid_argument("INSTALLATION_RECORD_FIELDS");
      const auto raw_id = r.at("anchor_id").get<int64_t>();
      if (!r.at("anchor_id").is_number_integer() || raw_id < 0 || raw_id > UINT32_MAX || !seen.insert(raw_id).second)
        throw std::invalid_argument("INSTALLATION_ID");
      input.ids.push_back(static_cast<uint32_t>(raw_id));
      input.offsets[i] = r.at("signed_offset_m").get<double>();
    }
    const auto& c = j.at("joint_covariance_m2");
    if (!c.is_array() || c.size() != n) throw std::invalid_argument("INSTALLATION_COVARIANCE_DIMENSION");
    input.covariance.resize(n, n);
    for (size_t i = 0; i < n; ++i) {
      if (!c[i].is_array() || c[i].size() != n) throw std::invalid_argument("INSTALLATION_COVARIANCE_DIMENSION");
      for (size_t k = 0; k < n; ++k) input.covariance(i, k) = c[i][k].get<double>();
    }
    if (!input.offsets.allFinite() || !input.covariance.allFinite() || !input.covariance.isApprox(input.covariance.transpose(), 1e-12) ||
        Eigen::LLT<Eigen::MatrixXd>(input.covariance).info() != Eigen::Success)
      throw std::invalid_argument("INSTALLATION_INVALID_NUMERICS");
    return input;
  }
  static InstallationInput load(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::invalid_argument("INSTALLATION_MISSING: " + path);
    return parse(nlohmann::json::parse(file));
  }
};
}  // namespace liuo_tunnel
