#pragma once
#include <liuo_tunnel/common/sensors.hpp>
#include <liuo_tunnel/uwb/static_anchor_edges.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Cholesky>
#include <filesystem>
#include <set>

namespace liuo_tunnel {
// Joint Euclidean A/L marginal. L uses mrad/metres, matching the well-scaled
// production state. Cross terms are mandatory; no independent anchor priors.
struct CalibratedMap {
  std::vector<uint32_t> ids;
  std::vector<gtsam::Point3> means;
  bool has_reference = true;
  gtsam::Vector2 reference = gtsam::Vector2::Zero();
  gtsam::Matrix covariance;
  nlohmann::json document;

  gtsam::KeyVector keys() const {
    gtsam::KeyVector result;
    for (auto id : ids) result.push_back(gtsam::Symbol('a', id));
    if (has_reference) result.push_back(gtsam::Symbol('l', 0));
    return result;
  }
  gtsam::Vector mean_vector() const {
    gtsam::Vector mean(3 * ids.size() + (has_reference ? 2 : 0));
    for (size_t i = 0; i < ids.size(); ++i) mean.segment<3>(3 * i) = means[i];
    if (has_reference) mean.tail<2>() = reference;
    return mean;
  }
  static CalibratedMap parse(const nlohmann::json& j, const nlohmann::json& sensor_identity) {
    const bool anchor_only = j.at("schema_version") == "liuo_anchor_map_v3";
    if (
      (!anchor_only && j.at("schema_version") != "liuo_joint_map_v1" && j.at("schema_version") != "liuo_joint_map_static_v2") || j.at("frame") != "map" ||
      j.at("anchor_units") != "m" || j.at("status") != "PROVISIONAL_INITIALIZED" || j.at("unresolved_branches") != nlohmann::json::array())
      throw std::invalid_argument("CALIBRATED_MAP_INVALID_SCHEMA_OR_BRANCHES");
    if (anchor_only) {
      if (j.contains("reference_units") || j.contains("reference_origin_m") || j.contains("reference_mean") || j.at("state_layout") != "A_only")
        throw std::invalid_argument("ANCHOR_ONLY_MAP_FORBIDS_GLOBAL_REFERENCE");
      const auto policy = j.at("static_information_policy").get<std::string>();
      if (policy == "AA_DISABLED") {
        if (j.contains("static_edge_ledger") || j.at("uwb_information").at("use_aa") != false || j.at("uwb_information").at("accepted_aa_measurements") != 0)
          throw std::invalid_argument("ANCHOR_ONLY_MAP_AA_POLICY_CONFLICT");
      } else if (policy != "EXACT_LEDGER" || !j.contains("static_edge_ledger"))
        throw std::invalid_argument("ANCHOR_ONLY_MAP_REQUIRES_EXACT_LEDGER");
    } else if (j.at("reference_units") != nlohmann::json::array({"mrad", "m"}) || j.at("reference_origin_m") != nlohmann::json::array({0, 0, 0}))
      throw std::invalid_argument("CALIBRATED_MAP_INVALID_REFERENCE");
    if (j.at("sensor_identity") != sensor_identity) throw std::invalid_argument("CALIBRATED_MAP_SENSOR_IDENTITY_CONFLICT");
    for (const char* field : {"map_id", "coordinate_gauge", "source_run", "approximation"})
      if (j.at(field).get<std::string>().empty()) throw std::invalid_argument("CALIBRATED_MAP_MISSING_PROVENANCE");
    if (
      !j.at("source_identity").is_object() || j.at("source_identity").empty() || !j.at("installation_provenance").is_object() || !j.at("not_initialized").is_array() ||
      !j.at("estimated_vehicle_bounds_m").is_array())
      throw std::invalid_argument("CALIBRATED_MAP_MISSING_SCOPE");
    const std::set<std::string> source_fields{"schema_version", "source", "config_sha256", "binary_sha256", "frozen_raw_sha256", "frozen_capture_sha256", "installation_sha256", "inherited_map_sha256"};
    const auto& identity = j.at("source_identity");
    if (identity.at("schema_version") != "liuo_map_source_identity_v1") throw std::invalid_argument("MAP_SOURCE_IDENTITY_SCHEMA");
    for (const auto& field : identity.items())
      if (!source_fields.count(field.key())) throw std::invalid_argument("MAP_SOURCE_IDENTITY_FORBIDDEN_FIELD");
    CalibratedMap m;
    m.document = j;
    m.has_reference = !anchor_only;
    std::set<uint32_t> seen;
    for (const auto& anchor : j.at("anchors")) {
      const auto raw = anchor.at("id").get<int64_t>();
      if (!anchor.at("id").is_number_integer() || raw < 0 || raw > UINT32_MAX || !seen.insert(raw).second) throw std::invalid_argument("CALIBRATED_MAP_INVALID_ID");
      m.ids.push_back(static_cast<uint32_t>(raw));
      m.means.push_back(vector3(anchor.at("mean_m")));
    }
    // V3 may checkpoint a partial map; this does not certify reuse observability.
    if (m.ids.size() < (anchor_only ? 1u : 3u) || m.ids.size() > (anchor_only ? 18u : 1000u)) throw std::invalid_argument("CALIBRATED_MAP_COUNT");
    if (!anchor_only) {
      const auto& ref = j.at("reference_mean");
      if (ref.size() != 2) throw std::invalid_argument("CALIBRATED_MAP_REFERENCE_DIMENSION");
      m.reference = gtsam::Vector2(ref[0].get<double>(), ref[1].get<double>());
    }
    const size_t n = 3 * m.ids.size() + (anchor_only ? 0 : 2);
    const auto& c = j.at("joint_covariance");
    if (c.size() != n) throw std::invalid_argument("CALIBRATED_MAP_COVARIANCE_DIMENSION");
    m.covariance.resize(n, n);
    for (size_t i = 0; i < n; ++i) {
      if (c[i].size() != n) throw std::invalid_argument("CALIBRATED_MAP_COVARIANCE_DIMENSION");
      for (size_t k = 0; k < n; ++k) m.covariance(i, k) = c[i][k].get<double>();
    }
    if (
      !m.mean_vector().allFinite() || !m.covariance.allFinite() || !m.covariance.isApprox(m.covariance.transpose(), 1e-10) ||
      Eigen::LLT<gtsam::Matrix>(m.covariance).info() != Eigen::Success)
      throw std::invalid_argument("CALIBRATED_MAP_NONFINITE_OR_NOT_SPD");
    if (j.at("schema_version") == "liuo_joint_map_static_v2" || (anchor_only && j.contains("static_edge_ledger"))) {
      const auto& ledger = j.at("static_edge_ledger");
      StaticAnchorEdges validated(ledger.at("network"), ledger.at("deployment_epoch"), 65536, ledger);
      for (const auto& leaf : ledger.at("leaves"))
        if (!seen.count(leaf.at("first").get<uint32_t>()) || !seen.count(leaf.at("second").get<uint32_t>())) throw std::invalid_argument("STATIC_MAP_LEDGER_UNKNOWN_ANCHOR");
    } else if (j.contains("static_edge_ledger"))
      throw std::invalid_argument("STATIC_MAP_LEDGER_REQUIRES_V2");
    return m;
  }
  static CalibratedMap load(const std::filesystem::path& path, const nlohmann::json& sensors) {
    std::ifstream input(path);
    if (!input) throw std::invalid_argument("CALIBRATED_MAP_MISSING");
    return parse(nlohmann::json::parse(input), sensors);
  }
  void save_atomic(const std::filesystem::path& path) const {
    std::filesystem::create_directories(path.parent_path());
    const auto temporary = path.string() + ".tmp";
    {
      std::ofstream output;
      output.exceptions(std::ios::failbit | std::ios::badbit);
      output.open(temporary);
      output << document.dump(2) << '\n';
      output.close();
    }
    std::filesystem::rename(temporary, path);
  }
};

class CalibratedMapPrior : public gtsam::NoiseModelFactor {
public:
  explicit CalibratedMapPrior(const CalibratedMap& map)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(map.covariance), map.keys()),
    mean_(map.mean_vector()),
    anchor_count_(map.ids.size()),
    has_reference_(map.has_reference) {}
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    gtsam::Vector residual = -mean_;
    if (H) H->clear();
    for (size_t i = 0; i < anchor_count_; ++i) {
      residual.segment<3>(3 * i) += values.at<gtsam::Point3>(keys()[i]);
      if (H) {
        H->push_back(gtsam::Matrix::Zero(mean_.size(), 3));
        H->back().block<3, 3>(3 * i, 0).setIdentity();
      }
    }
    if (has_reference_) {
      residual.tail<2>() += values.at<gtsam::Vector2>(keys().back());
      if (H) {
        H->push_back(gtsam::Matrix::Zero(mean_.size(), 2));
        H->back().bottomRows<2>().setIdentity();
      }
    }
    return residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new CalibratedMapPrior(*this)); }

private:
  gtsam::Vector mean_;
  size_t anchor_count_;
  bool has_reference_;
};
}  // namespace liuo_tunnel
