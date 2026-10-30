// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>

namespace onero_a1_kinematics
{
struct Profile
{
  Profile() = default;
  Profile(Eigen::VectorXd reference, Eigen::VectorXd weights);
  Eigen::VectorXd reference, weights, lower, upper;
  bool enabled = true;
  double gain = .8, max_speed = .2, limit_margin = .15, limit_weight = 1.;
  double continuity_weight = .15, path_weight = .5;
  int refinement_steps = 3, planning_candidates = 2;
  double max_wrist_roll_deviation = 1.5707963267948966;
  void setBounds(const Eigen::VectorXd& minimum, const Eigen::VectorXd& maximum);
  double cost(const Eigen::VectorXd& q) const;
  Eigen::VectorXd gradient(const Eigen::VectorXd& q) const;
  Eigen::VectorXd nullspaceVelocity(const Eigen::VectorXd& q, const Eigen::MatrixXd& jacobian,
                                   double scale = 1.) const;
  double pathCost(const Eigen::MatrixXd& positions) const;
};
Profile loadProfile(const std::vector<std::string>& joints);
Eigen::VectorXd projectNullspace(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& vector);
}  // namespace onero_a1_kinematics
