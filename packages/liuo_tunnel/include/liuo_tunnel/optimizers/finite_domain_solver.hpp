#pragma once
#include <liuo_tunnel/uwb/local_installation.hpp>
#include <gtsam_points/optimizers/gaussian_factor_graph_solver.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext_params.hpp>
#include <gtsam/linear/JacobianFactor.h>
#include <Eigen/Cholesky>
#include <set>

namespace liuo_tunnel {
// Numerical inequalities for a damped step, never measurement factors or priors.
struct StepInequality {
  gtsam::VectorValues row;
  double bound;
  double dot(const gtsam::VectorValues& point) const {
    double result = 0;
    for (const auto& term : row) result += term.second.dot(point.at(term.first));
    return result;
  }
};
inline gtsam::VectorValues constrained_gaussian_step(const gtsam::GaussianFactorGraph& graph, const gtsam::Ordering& ordering, const std::vector<StepInequality>& constraints) {
  const auto unconstrained = graph.optimize(ordering);
  auto current = unconstrained;
  if (constraints.empty()) return current;
  for (auto& term : current) term.second.setZero();
  for (const auto& c : constraints)
    if (!std::isfinite(c.bound) || c.bound < 0 || !std::isfinite(c.row.norm()) || c.row.norm() <= 0) throw std::invalid_argument("INVALID_STEP_DOMAIN");
  std::vector<size_t> active;
  // Primal feasible active set: move only as far as the first blocking plane.
  // This also avoids adding inconsistent parallel equalities in intermediate solves.
  for (size_t iteration = 0; iteration < 256; ++iteration) {
    auto system = graph;
    for (size_t index : active) {
      const auto& c = constraints[index];
      std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
      for (const auto& term : c.row) terms.emplace_back(term.first, term.second.transpose());
      system.emplace_shared<gtsam::JacobianFactor>(terms, gtsam::Vector1(c.bound), gtsam::noiseModel::Constrained::All(1));
    }
    const auto optimum = active.empty() ? unconstrained : system.optimize(ordering, gtsam::EliminateQR);
    const auto direction = optimum - current;
    double fraction = 1.;
    size_t blocker = constraints.size();
    for (size_t i = 0; i < constraints.size(); ++i) {
      if (std::find(active.begin(), active.end(), i) != active.end()) continue;
      const auto& c = constraints[i];
      const double change = c.dot(direction);
      if (change <= 1e-12) continue;
      const double allowed = std::max(0., c.bound - c.dot(current)) / change;
      if (allowed < fraction) {
        fraction = allowed;
        blocker = i;
      }
    }
    current += fraction * direction;
    if (blocker != constraints.size()) {
      active.push_back(blocker);
      continue;
    }
    if (!active.empty()) {
      const auto gradient = graph.gradient(current);
      gtsam::Matrix gram(active.size(), active.size());
      gtsam::Vector rhs(active.size());
      for (size_t i = 0; i < active.size(); ++i) {
        const auto& a = constraints[active[i]];
        rhs[i] = -a.dot(gradient);
        for (size_t j = 0; j < active.size(); ++j) {
          const auto& b = constraints[active[j]];
          double product = 0;
          for (const auto& term : a.row)
            if (b.row.exists(term.first)) product += term.second.dot(b.row.at(term.first));
          gram(i, j) = product;
        }
      }
      Eigen::LDLT<gtsam::Matrix> decomposition(gram);
      if (decomposition.info() != Eigen::Success || decomposition.vectorD().minCoeff() <= 1e-12) throw std::runtime_error("DEPENDENT_STEP_DOMAIN_CONSTRAINTS");
      const gtsam::Vector multipliers = decomposition.solve(rhs);
      Eigen::Index lowest;
      if (multipliers.minCoeff(&lowest) < -1e-8) {
        active.erase(active.begin() + lowest);
        continue;
      }
    }
    for (const auto& c : constraints)
      if (c.dot(current) > c.bound + 1e-8) throw std::runtime_error("INFEASIBLE_DOMAIN_STEP");
    return current;
  }
  throw std::runtime_error("STEP_DOMAIN_ITERATION_LIMIT");
}

class FiniteDomainSolver : public gtsam_points::GaussianFactorGraphSolver {
public:
  explicit FiniteDomainSolver(const gtsam::NonlinearFactorGraph& graph) {
    std::set<gtsam::Key> anchors;
    for (const auto& factor : graph)
      if (auto local = std::dynamic_pointer_cast<LocalInstallationFactor>(factor)) {
        // Production attaches one finite installation observation per anchor.
        // Reject ambiguous projection order rather than silently dropping a domain.
        if (!anchors.insert(local->key2()).second) throw std::invalid_argument("MULTIPLE_LOCAL_DOMAINS_FOR_ANCHOR");
        factors_.push_back(local);
      }
  }
  bool empty() const { return factors_.empty(); }
  void linearization(const gtsam::Values& values) {
    reference_ = values;
    constraints_.clear();
    for (const auto& factor : factors_) {
      const auto d = factor->linearized_domain(values);
      for (double sign : {-1., 1.}) {
        StepInequality c;
        c.row.insert(factor->key1(), sign * d.pose_jacobian.transpose());
        c.row.insert(factor->key2(), sign * d.anchor_jacobian.transpose());
        const double remaining = sign > 0 ? d.upper - d.position : d.position - d.lower;
        // A tiny inward reserve addresses linear-solve roundoff. It only shrinks
        // the domain; nonlinear curvature is still checked by original graph.error.
        c.bound = remaining - std::min(1e-8, remaining * .5);
        const double norm = c.row.norm();
        c.row *= 1. / norm;
        c.bound /= norm;
        constraints_.push_back(std::move(c));
      }
    }
  }
  gtsam::VectorValues solve(const gtsam::GaussianFactorGraph& graph, const gtsam::Ordering& ordering) override {
    const auto step = constrained_gaussian_step(graph, ordering, constraints_);
    auto trial = reference_.retract(step);
    for (const auto& factor : factors_) factor->restore_trial_domain(trial);
    const auto restored = reference_.localCoordinates(trial);
    // The caller retracts this returned step again. Check that exact path,
    // including its floating-point round trip, before evaluating any likelihood.
    const auto checked = reference_.retract(restored);
    for (const auto& factor : factors_) factor->linearized_domain(checked);
    return restored;
  }

private:
  std::vector<std::shared_ptr<LocalInstallationFactor>> factors_;
  std::vector<StepInequality> constraints_;
  gtsam::Values reference_;
};
inline gtsam_points::LevenbergMarquardtExtParams finite_domain_parameters(const gtsam::NonlinearFactorGraph& graph, const gtsam_points::LevenbergMarquardtExtParams& original) {
  auto solver = std::make_shared<FiniteDomainSolver>(graph);
  if (solver->empty()) return original;
  if (original.solver) throw std::invalid_argument("FINITE_DOMAIN_CUSTOM_SOLVER_CONFLICT");
  auto parameters = original;
  parameters.solver = solver;
  return parameters;
}
}  // namespace liuo_tunnel
