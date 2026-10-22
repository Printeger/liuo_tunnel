#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace liuo_tunnel_sim {
// Counter-based streams: enabling a different sensor/link cannot consume this stream.
inline uint64_t mix(uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
inline double normal(uint64_t seed, uint64_t stream, uint64_t sample, uint64_t component = 0) {
  auto key = mix(seed) ^ mix(stream + 1000) ^ mix(sample + 1000000) ^ mix(component + 1000000000);
  const double u = ((mix(key) >> 11) + 0.5) / 9007199254740992.0;
  const double v = ((mix(key + 1) >> 11) + 0.5) / 9007199254740992.0;
  return std::sqrt(-2 * std::log(u)) * std::cos(2 * M_PI * v);
}
struct Motion {
  Eigen::Vector3d p, v, a;
  Eigen::Matrix3d R;
  Eigen::Vector3d omega;
};
// Continuous acceleration half-cosine ramps: vmax=5, ramp=10s, amax=pi/4 < 1.
inline Motion motion(double seconds, double radius = 0) {
  double t = std::max(0.0, seconds - 5), x = 0, v = 0, a = 0;
  for (double goal : {600., 1200., 1800., 2350.}) {
    double length = goal - x, cruise = (length - 50) / 5, travel = 20 + cruise;
    if (t <= travel) {
      if (t < 10) {
        v = 2.5 * (1 - std::cos(M_PI * t / 10));
        a = M_PI / 4 * std::sin(M_PI * t / 10);
        x += 2.5 * (t - 10 / M_PI * std::sin(M_PI * t / 10));
      } else if (t < 10 + cruise) {
        x += 25 + 5 * (t - 10);
        v = 5;
      } else {
        double u = t - 10 - cruise;
        x += 25 + 5 * cruise + 2.5 * (u + 10 / M_PI * std::sin(M_PI * u / 10));
        v = 2.5 * (1 + std::cos(M_PI * u / 10));
        a = -M_PI / 4 * std::sin(M_PI * u / 10);
      }
      break;
    }
    x = goal;
    t -= travel;
    if (t < 5) break;
    t -= 5;
  }
  Motion m;
  m.R = Eigen::Matrix3d::Identity();
  m.omega.setZero();
  m.p = {x, 0, 1.2};
  m.v = {v, 0, 0};
  m.a = {a, 0, 0};
  if (radius > 0) {
    double theta = x / radius;
    m.R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    m.p = {radius * std::sin(theta), radius * (1 - std::cos(theta)), 1.2};
    m.v = m.R * Eigen::Vector3d(v, 0, 0);
    m.a = m.R * Eigen::Vector3d(a, v * v / radius, 0);
    m.omega.z() = v / radius;
  }
  return m;
}
// Straight rail round trip: finish outbound at 530 s, stand for 5 s, then
// reverse along the track with unchanged body heading. Return ends at 1060 s.
// Backward motion is generated physically; no rail constraint enters estimation.
inline Motion rail_motion(double seconds, bool return_trip = false) {
  if (!return_trip || seconds <= 530) return motion(seconds);
  auto m = motion(seconds - 530);
  m.p.x() = 2350 - m.p.x();
  m.v.x() = -m.v.x();
  m.a.x() = -m.a.x();
  return m;
}
// Simulation-only representative starts; translation follows the existing world axis.
// Body yaw is independent of travel direction (no nonholonomic assumption).
inline Motion initialized_motion(double seconds, bool return_trip, const Eigen::Vector3d& start, double yaw) {
  auto m = rail_motion(seconds, return_trip);
  m.p += start - Eigen::Vector3d(0, 0, 1.2);
  m.R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return m;
}
// Adapted physical definition from quadToIapImuMsg, IAP quadrotor_simulator_so3.cpp @ fbccae168.
inline Eigen::Vector3d specific_force(const Motion& m) {
  return m.R.transpose() * (m.a - Eigen::Vector3d(0, 0, -9.80665));
}
// Deployment v5: seven anchors at 400 m spacing; at most three per closed 1 km.
// Coordinates belong only to the simulator; the estimator loads its external map.
inline constexpr int anchor_count = 7;
inline Eigen::Vector3d anchor(int i) {
  return {400.0 * i, (i % 4 == 0 || i % 4 == 3) ? 2.8 : -2.8, i % 4 < 2 ? 4.6 : .4};
}
// Reserve one in ten exchange slots for AA to prevent starvation under TA overload.
inline bool prefer_anchor_exchange(uint64_t slot) {
  return slot % 10 == 9;
}
inline std::pair<Eigen::Vector3d, Eigen::Vector3d> cabinet_bounds(int k) {
  double center = 3.1 + k * 5.7 + std::sin(k * 1.7);
  double lo = std::ceil((center - .4) * 10 - .5) * .1, hi = std::ceil((center + .4) * 10 - .5) * .1;
  return {{lo, k % 2 ? 2.2 : -3.0, .3}, {hi, k % 2 ? 3.0 : -2.2, 2.3}};
}
inline bool line_of_sight(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  if (std::abs(a.y()) >= 3 || std::abs(b.y()) >= 3 || a.z() <= 0 || b.z() <= 0 || a.z() >= 5 || b.z() >= 5) return false;
  // Slab intersections against the same voxel-aligned cabinets used by LiDAR.
  for (int k = 0; k < 8; k++) {
    auto box = cabinet_bounds(k);
    double near = 0, far = 1;
    bool intersects = true;
    for (int j = 0; j < 3; j++) {
      double d = b[j] - a[j];
      if (std::abs(d) < 1e-12) {
        if (a[j] < box.first[j] || a[j] > box.second[j]) intersects = false;
        continue;
      }
      double u = (box.first[j] - a[j]) / d, v = (box.second[j] - a[j]) / d;
      if (u > v) std::swap(u, v);
      near = std::max(near, u);
      far = std::min(far, v);
    }
    if (intersects && near <= far) return false;
  }
  return true;
}
inline bool reachable(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return (a - b).norm() <= 800.0;
}
inline double range(const Eigen::Vector3d& a, const Eigen::Vector3d& b, uint64_t seed, uint64_t stream, uint64_t index) {
  if (!reachable(a, b)) return std::numeric_limits<double>::quiet_NaN();
  return (a - b).norm() + 0.10 * normal(seed, stream, index);
}
}  // namespace liuo_tunnel_sim
