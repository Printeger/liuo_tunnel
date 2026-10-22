#pragma once
#include <liuo_tunnel/common/sensors.hpp>
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
  gtsam::Vector2 reference;
  gtsam::Matrix covariance;
  nlohmann::json document;

  gtsam::KeyVector keys() const {
    gtsam::KeyVector result;
    for (auto id : ids) result.push_back(gtsam::Symbol('a', id));
    result.push_back(gtsam::Symbol('l', 0));
    return result;
  }
  gtsam::Vector mean_vector() const {
    gtsam::Vector mean(3 * ids.size() + 2);
    for (size_t i = 0; i < ids.size(); ++i) mean.segment<3>(3 * i) = means[i];
    mean.tail<2>() = reference;
    return mean;
  }
  static CalibratedMap parse(const nlohmann::json& j, const nlohmann::json& sensor_identity) {
    if (j.at("schema_version") != "liuo_joint_map_v1" || j.at("frame") != "map" || j.at("anchor_units") != "m" ||
        j.at("reference_units") != nlohmann::json::array({"mrad", "m"}) || j.at("reference_origin_m") != nlohmann::json::array({0, 0, 0}) ||
        j.at("status") != "PROVISIONAL_INITIALIZED" || j.at("unresolved_branches") != nlohmann::json::array())
      throw std::invalid_argument("CALIBRATED_MAP_INVALID_SCHEMA_OR_BRANCHES");
    if (j.at("sensor_identity") != sensor_identity) throw std::invalid_argument("CALIBRATED_MAP_SENSOR_IDENTITY_CONFLICT");
    for (const char* field : {"map_id", "coordinate_gauge", "source_run", "approximation"})
      if (j.at(field).get<std::string>().empty()) throw std::invalid_argument("CALIBRATED_MAP_MISSING_PROVENANCE");
    if (!j.at("source_identity").is_object() || j.at("source_identity").empty() || !j.at("installation_provenance").is_object() ||
        !j.at("not_initialized").is_array() || !j.at("estimated_vehicle_bounds_m").is_array())
      throw std::invalid_argument("CALIBRATED_MAP_MISSING_SCOPE");
    const std::set<std::string> source_fields{"schema_version", "source", "config_sha256", "binary_sha256", "frozen_raw_sha256", "frozen_capture_sha256", "installation_sha256"};
    const auto& identity = j.at("source_identity");
    if (identity.at("schema_version") != "liuo_map_source_identity_v1") throw std::invalid_argument("MAP_SOURCE_IDENTITY_SCHEMA");
    for (const auto& field : identity.items())
      if (!source_fields.count(field.key())) throw std::invalid_argument("MAP_SOURCE_IDENTITY_FORBIDDEN_FIELD");
    CalibratedMap m;
    m.document = j;
    std::set<uint32_t> seen;
    for (const auto& anchor : j.at("anchors")) {
      const auto raw = anchor.at("id").get<int64_t>();
      if (!anchor.at("id").is_number_integer() || raw < 0 || raw > UINT32_MAX || !seen.insert(raw).second)
        throw std::invalid_argument("CALIBRATED_MAP_INVALID_ID");
      m.ids.push_back(static_cast<uint32_t>(raw));
      m.means.push_back(vector3(anchor.at("mean_m")));
    }
    // Current reuse initializer needs three non-collinear measured anchors;
    // this is an interface constraint, not a claim of universal observability.
    if (m.ids.size() < 3 || m.ids.size() > 1000) throw std::invalid_argument("CALIBRATED_MAP_COUNT");
    const auto& ref = j.at("reference_mean");
    if (ref.size() != 2) throw std::invalid_argument("CALIBRATED_MAP_REFERENCE_DIMENSION");
    m.reference = gtsam::Vector2(ref[0].get<double>(), ref[1].get<double>());
    const size_t n = 3 * m.ids.size() + 2;
    const auto& c = j.at("joint_covariance");
    if (c.size() != n) throw std::invalid_argument("CALIBRATED_MAP_COVARIANCE_DIMENSION");
    m.covariance.resize(n, n);
    for (size_t i = 0; i < n; ++i) {
      if (c[i].size() != n) throw std::invalid_argument("CALIBRATED_MAP_COVARIANCE_DIMENSION");
      for (size_t k = 0; k < n; ++k) m.covariance(i, k) = c[i][k].get<double>();
    }
    if (!m.mean_vector().allFinite() || !m.covariance.allFinite() || !m.covariance.isApprox(m.covariance.transpose(), 1e-10) ||
        Eigen::LLT<gtsam::Matrix>(m.covariance).info() != Eigen::Success)
      throw std::invalid_argument("CALIBRATED_MAP_NONFINITE_OR_NOT_SPD");
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
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(map.covariance), map.keys()), mean_(map.mean_vector()) {}
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    gtsam::Vector residual = -mean_;
    if (H) H->clear();
    for (size_t i = 0; i + 1 < keys().size(); ++i) {
      residual.segment<3>(3 * i) += values.at<gtsam::Point3>(keys()[i]);
      if (H) {
        H->push_back(gtsam::Matrix::Zero(mean_.size(), 3));
        H->back().block<3, 3>(3 * i, 0).setIdentity();
      }
    }
    residual.tail<2>() += values.at<gtsam::Vector2>(keys().back());
    if (H) {
      H->push_back(gtsam::Matrix::Zero(mean_.size(), 2));
      H->back().bottomRows<2>().setIdentity();
    }
    return residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new CalibratedMapPrior(*this)); }
private:
  gtsam::Vector mean_;
};
}  // namespace liuo_tunnel
