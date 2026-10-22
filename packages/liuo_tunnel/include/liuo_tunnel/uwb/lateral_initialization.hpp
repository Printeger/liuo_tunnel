#pragma once
#include <liuo_tunnel/uwb/known_map.hpp>
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <random>
namespace liuo_tunnel {
struct LateralReference {
  Eigen::Vector2d normal;
  double offset;
  Eigen::Matrix2d covariance;  // angle, offset; joint local linear approximation
  double whitened_cost;
  static LateralReference fit(const KnownMap& map) {
    const auto& j = map.installation;
    if (
      j.is_null() || j.at("schema_version") != "liuo_lateral_installation_v1" || j.at("units") != "m" || j.at("reference_model") != "shared_straight_horizontal_axis" ||
      j.at("reference_id").get<std::string>().empty() || j.at("provenance").get<std::string>().empty() || j.at("positive_side_convention").get<std::string>().empty() ||
      j.at("independent_from_map_survey") != true)
      throw std::invalid_argument("INVALID_LATERAL_INSTALLATION_REFERENCE");
    const auto& records = j.at("records");
    const size_t count = records.size();
    if (count < 3 || count > map.ids.size()) throw std::invalid_argument("INVALID_LATERAL_INSTALLATION_COUNT");
    Eigen::MatrixXd design(count, 3), C(count, count);
    Eigen::VectorXd observed(count);
    std::vector<size_t> indices;
    std::set<uint32_t> used;
    const auto& covariance = j.at("joint_covariance_m2");
    if (covariance.size() != count) throw std::invalid_argument("INSTALLATION_COVARIANCE_DIMENSION");
    for (size_t i = 0; i < count; ++i) {
      const uint32_t id = records[i].at("anchor_id");
      if (!used.insert(id).second) throw std::invalid_argument("DUPLICATE_INSTALLATION_ID");
      indices.push_back(map.index(id));
      const auto& a = map.means[indices.back()];
      design.row(i) << a.x(), a.y(), -1;
      observed[i] = records[i].at("signed_offset_m");
      if (covariance[i].size() != count) throw std::invalid_argument("INSTALLATION_COVARIANCE_DIMENSION");
      for (size_t k = 0; k < count; ++k) C(i, k) = covariance[i][k].get<double>();
    }
    if (!observed.allFinite() || !C.allFinite() || !C.isApprox(C.transpose(), 1e-12) || Eigen::LLT<Eigen::MatrixXd>(C).info() != Eigen::Success)
      throw std::invalid_argument("INVALID_INSTALLATION_COVARIANCE");
    auto decomposition = design.colPivHouseholderQr();
    if (decomposition.rank() != 3) throw std::invalid_argument("INSTALLATION_AXIS_UNOBSERVABLE");
    const Eigen::Vector3d guess = decomposition.solve(observed);
    if (std::abs(guess.head<2>().norm() - 1) > .2) throw std::invalid_argument("INSTALLATION_REFERENCE_MODEL_CONFLICT");
    double angle = std::atan2(guess[1], guess[0]), offset = guess[2] / guess.head<2>().norm();
    Eigen::Matrix2d information;
    Eigen::MatrixXd total;
    Eigen::VectorXd residual;
    for (size_t iteration = 0; iteration < 10; ++iteration) {
      Eigen::Vector2d n(std::cos(angle), std::sin(angle)), tangent(-n.y(), n.x());
      total = C;
      if (map.surveyed())
        for (size_t i = 0; i < count; ++i)
          for (size_t k = 0; k < count; ++k) total(i, k) += n.dot(map.covariance.block<2, 2>(3 * indices[i], 3 * indices[k]) * n);
      Eigen::MatrixXd J(count, 2);
      residual.resize(count);
      for (size_t i = 0; i < count; ++i) {
        const Eigen::Vector2d a = map.means[indices[i]].head<2>();
        residual[i] = n.dot(a) - offset - observed[i];
        J.row(i) << tangent.dot(a), -1;
      }
      Eigen::LLT<Eigen::MatrixXd> solver(total);
      information = J.transpose() * solver.solve(J);
      Eigen::LLT<Eigen::Matrix2d> normal_solver(information);
      if (normal_solver.info() != Eigen::Success) throw std::invalid_argument("INSTALLATION_AXIS_UNOBSERVABLE");
      const Eigen::Vector2d step = -normal_solver.solve(J.transpose() * solver.solve(residual));
      angle += step[0];
      offset += step[1];
      if (step.norm() < 1e-10) break;
    }
    return {{std::cos(angle), std::sin(angle)}, offset, information.inverse(), residual.dot(total.llt().solve(residual))};
  }
};
struct LocalWallPair {
  Eigen::Vector2d normal;
  double center_offset, width, residual_rms;
};
// Actual gravity-leveled cloud only. No world boundary, fixed height or yaw.
inline LocalWallPair extract_local_walls(const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 100) throw std::invalid_argument("INSUFFICIENT_WALL_POINTS");
  std::vector<Eigen::Vector2d> remaining;
  const size_t stride = std::max(size_t(1), points.size() / 4096);
  for (size_t i = 0; i < points.size(); i += stride)
    if (points[i].allFinite()) remaining.push_back(points[i].head<2>());
  std::mt19937 rng(0);
  std::array<Eigen::Vector2d, 2> normals;
  std::array<double, 2> offsets, rms;
  for (size_t plane = 0; plane < 2; ++plane) {
    std::vector<size_t> best;
    if (remaining.size() < 100) throw std::invalid_argument("WALL_PAIR_NOT_OBSERVED");
    for (size_t trial = 0; trial < 128; ++trial) {
      const auto& a = remaining[rng() % remaining.size()];
      Eigen::Vector2d direction = remaining[rng() % remaining.size()] - a;
      if (direction.norm() < 1) continue;
      Eigen::Vector2d normal(-direction.y(), direction.x());
      normal.normalize();
      const double offset = normal.dot(a);
      std::vector<size_t> support;
      for (size_t i = 0; i < remaining.size(); ++i)
        if (std::abs(normal.dot(remaining[i]) - offset) < .06) support.push_back(i);
      if (support.size() > best.size()) best = std::move(support);
    }
    if (best.size() < 100) throw std::invalid_argument("WALL_PAIR_NOT_OBSERVED");
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (auto i : best) center += remaining[i];
    center /= best.size();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (auto i : best) {
      const auto delta = (remaining[i] - center).eval();
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance / best.size());
    normals[plane] = solver.eigenvectors().col(0);
    if (plane == 0 && normals[0].x() < 0) normals[0] *= -1;
    if (plane == 1 && normals[0].dot(normals[1]) < 0) normals[1] *= -1;
    offsets[plane] = normals[plane].dot(center);
    rms[plane] = std::sqrt(std::max(0., solver.eigenvalues()[0]));
    std::vector<Eigen::Vector2d> other;
    size_t cursor = 0;
    for (size_t i = 0; i < remaining.size(); ++i) {
      if (cursor < best.size() && best[cursor] == i)
        ++cursor;
      else
        other.push_back(remaining[i]);
    }
    remaining = std::move(other);
  }
  if (normals[0].dot(normals[1]) < .995 || offsets[0] * offsets[1] >= 0) throw std::invalid_argument("WALL_PAIR_GEOMETRY_CONFLICT");
  return {(normals[0] + normals[1]).normalized(), (offsets[0] + offsets[1]) / 2, std::abs(offsets[0] - offsets[1]), std::max(rms[0], rms[1])};
}
inline std::vector<gtsam::Pose3>
lateral_pose_seeds(const std::vector<gtsam::Point3>& tags, const gtsam::Rot3& gravity, const gtsam::Vector3& lever, const LocalWallPair& walls, const LateralReference& reference) {
  const double yaw = std::atan2(reference.normal.y(), reference.normal.x()) - std::atan2(walls.normal.y(), walls.normal.x());
  std::vector<gtsam::Pose3> seeds;
  for (int direction = 0; direction < 2; ++direction) {
    const auto rotation = gtsam::Rot3::Rz(yaw + direction * std::acos(-1.)) * gravity;
    const double target = reference.offset - (direction == 0 ? 1 : -1) * walls.center_offset;
    for (const auto& tag : tags) {
      gtsam::Point3 position = tag - rotation * lever;
      position.head<2>() += reference.normal * (target - reference.normal.dot(position.head<2>()));
      seeds.emplace_back(rotation, position);
    }
  }
  return seeds;
}
}  // namespace liuo_tunnel
