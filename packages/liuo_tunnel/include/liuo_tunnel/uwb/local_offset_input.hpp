#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
namespace liuo_tunnel {
// The frozen L scene's coordinate-free optional metadata. Exact synthetic
// offsets are constants, not fabricated field calibration measurements.
struct SyntheticLocalOffsets {
  nlohmann::json source;
  std::map<uint32_t, double> offsets;
  static SyntheticLocalOffsets parse(const nlohmann::json& j) {
    if (!j.is_object() || j.size() != 3 || !j.contains("source") || !j.contains("definition") || !j.contains("anchors"))
      throw std::invalid_argument("LOCAL_INSTALLATION_FORBIDDEN_FIELDS");
    if (
      j.at("source") != "exact synthetic relation; not field calibrated; no covariance or algorithm prior" ||
      j.at("definition") != "horizontal signed normal offset from design axis to antenna phase center, metres; valid ordinary wall band")
      throw std::invalid_argument("LOCAL_INSTALLATION_UNSUPPORTED_PROVENANCE");
    if (!j.at("anchors").is_array() || j.at("anchors").empty() || j.at("anchors").size() > 18) throw std::invalid_argument("LOCAL_INSTALLATION_CAPACITY");
    SyntheticLocalOffsets result;
    result.source = j;
    for (const auto& r : j.at("anchors")) {
      if (!r.is_object() || r.size() != 3 || !r.contains("id") || !r.contains("side") || !r.contains("lateral_offset_m"))
        throw std::invalid_argument("LOCAL_INSTALLATION_FORBIDDEN_RECORD_FIELDS");
      if (!r.at("id").is_number_integer()) throw std::invalid_argument("LOCAL_INSTALLATION_ID");
      const auto id = r.at("id").get<int64_t>();
      const double offset = r.at("lateral_offset_m").get<double>();
      const auto side = r.at("side").get<std::string>();
      if (
        id < 0 || id > UINT32_MAX || result.offsets.count(id) || !std::isfinite(offset) || offset == 0 || std::abs(offset) > 20 || (side != "left" && side != "right") ||
        ((side == "left") != (offset > 0)))
        throw std::invalid_argument("LOCAL_INSTALLATION_ID_SIDE_OR_OFFSET");
      result.offsets.emplace(static_cast<uint32_t>(id), offset);
    }
    return result;
  }
  static SyntheticLocalOffsets load(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::invalid_argument("LOCAL_INSTALLATION_MISSING");
    return parse(nlohmann::json::parse(input));
  }
};
}  // namespace liuo_tunnel
