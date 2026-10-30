// SPDX-License-Identifier: Apache-2.0
#include <onero_a1_kinematics/profile.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <regex>
#include <stdexcept>

namespace onero_a1_kinematics
{
Profile::Profile(Eigen::VectorXd ref, Eigen::VectorXd weight) : reference(std::move(ref)), weights(std::move(weight))
{
  if (!reference.size() || reference.size() != weights.size() || !reference.allFinite() || !weights.allFinite() ||
      (weights.array() < 0.).any() || weights.sum() <= 0.)
    throw std::invalid_argument("Invalid posture reference or weights");
}
void Profile::setBounds(const Eigen::VectorXd& minimum, const Eigen::VectorXd& maximum)
{
  if (minimum.size() != reference.size() || maximum.size() != reference.size() || !minimum.allFinite() ||
      !maximum.allFinite() || (minimum.array() >= maximum.array()).any() ||
      (reference.array() < minimum.array()).any() || (reference.array() > maximum.array()).any())
    throw std::invalid_argument("Posture reference must lie inside joint bounds");
  lower = minimum; upper = maximum;
}
double Profile::cost(const Eigen::VectorXd& q) const
{
  if (q.size() != reference.size() || !q.allFinite())
    throw std::invalid_argument("Invalid posture joint vector");
  double result = ((q-reference).array().square()*weights.array()).sum();
  if (lower.size())
    for (Eigen::Index i=0; i<q.size(); ++i)
    {
      const double distance = std::min(q[i]-lower[i],upper[i]-q[i]);
      const double violation = std::max(0.,1.-distance/limit_margin);
      result += limit_weight*violation*violation;
    }
  return result;
}
Eigen::VectorXd Profile::gradient(const Eigen::VectorXd& q) const
{
  cost(q);  // Dimension/finite validation shared with scoring.
  Eigen::VectorXd result = 2.*weights.cwiseProduct(q-reference);
  if (lower.size())
    for (Eigen::Index i=0; i<q.size(); ++i)
    {
      const double low = q[i]-lower[i], high = upper[i]-q[i];
      const double distance = std::min(low,high);
      if (distance < limit_margin)
        result[i] += (low < high ? -1. : 1.)*2.*limit_weight*(limit_margin-distance)/(limit_margin*limit_margin);
    }
  return result;
}
Eigen::VectorXd Profile::nullspaceVelocity(const Eigen::VectorXd& q, const Eigen::MatrixXd& jacobian, double scale) const
{
  const Eigen::VectorXd g = gradient(q);
  if (jacobian.cols() != q.size() || !jacobian.rows() || !jacobian.allFinite() || !std::isfinite(scale) || scale < 0.)
    throw std::invalid_argument("Invalid posture task Jacobian or scale");
  if (!enabled || scale == 0.)
    return Eigen::VectorXd::Zero(q.size());
  Eigen::VectorXd result = projectNullspace(jacobian,-gain*scale*g);
  result *= std::min(1.,max_speed/std::max(1e-12,result.cwiseAbs().maxCoeff()));
  return result;
}
Eigen::VectorXd projectNullspace(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& vector)
{
  if (!jacobian.rows() || jacobian.cols()!=vector.size() || !jacobian.allFinite() || !vector.allFinite())
    throw std::invalid_argument("Invalid nullspace projection input");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,Eigen::ComputeFullV);
  const double tolerance = 1e-6*std::max(1.,svd.singularValues()[0]);
  const Eigen::Index rank = (svd.singularValues().array()>tolerance).count();
  if (rank == vector.size())
    return Eigen::VectorXd::Zero(vector.size());
  const Eigen::MatrixXd basis = svd.matrixV().rightCols(vector.size()-rank);
  return basis*(basis.transpose()*vector);
}
double Profile::pathCost(const Eigen::MatrixXd& positions) const
{
  if (!positions.rows() || positions.cols() != reference.size() || !positions.allFinite())
    throw std::invalid_argument("Invalid posture path");
  double result=0.;
  for (Eigen::Index i=1; i<positions.rows(); ++i)
  {
    const Eigen::VectorXd a=positions.row(i-1).transpose(), b=positions.row(i).transpose();
    result += (b-a).norm()*(1.+path_weight*.5*(cost(a)+cost(b)));
  }
  return result;
}
Profile loadProfile(const std::vector<std::string>& joints)
{
  const auto file=ament_index_cpp::get_package_share_directory("onero_a1_kinematics")+"/config/posture.yaml";
  const auto config=YAML::LoadFile(file);
  const auto right=config["reference_right"].as<std::vector<double>>();
  const auto left=config["reference_left"].as<std::vector<double>>();
  const auto weight=config["weights"].as<std::vector<double>>();
  if (right.size()!=7 || left.size()!=7 || weight.size()!=7)
    throw std::invalid_argument("Posture configuration must contain seven joints per arm");
  Eigen::VectorXd ref(joints.size()), weights(joints.size());
  const std::regex name("joint([1-7])-a1_([lr])$");
  for (std::size_t i=0; i<joints.size(); ++i)
  {
    std::smatch match;
    if (!std::regex_search(joints[i],match,name))
      throw std::invalid_argument("No posture reference for joint "+joints[i]);
    const int index=std::stoi(match[1])-1;
    ref[i]=(match[2]=="l" ? left : right)[index]; weights[i]=weight[index];
  }
  Profile result(ref,weights);
  result.enabled=config["enabled"].as<bool>();result.gain=config["gain"].as<double>();
  result.max_speed=config["max_speed"].as<double>();result.limit_margin=config["limit_margin"].as<double>();
  result.limit_weight=config["limit_weight"].as<double>();result.continuity_weight=config["continuity_weight"].as<double>();
  result.path_weight=config["path_weight"].as<double>();result.refinement_steps=config["ik_refinement_steps"].as<int>();
  result.planning_candidates=config["planning_candidates"].as<int>();
  result.max_wrist_roll_deviation=config["max_wrist_roll_deviation"].as<double>(1.5707963267948966);
  if (!std::isfinite(result.gain) || result.gain<0. || !std::isfinite(result.max_speed) || result.max_speed<=0. ||
      !std::isfinite(result.limit_margin) || result.limit_margin<=0. || !std::isfinite(result.limit_weight) || result.limit_weight<0. ||
      !std::isfinite(result.continuity_weight) || result.continuity_weight<0. || !std::isfinite(result.path_weight) || result.path_weight<0. ||
      result.refinement_steps<0 || result.refinement_steps>10 || result.planning_candidates<1 || result.planning_candidates>3 ||
      !std::isfinite(result.max_wrist_roll_deviation) || result.max_wrist_roll_deviation<=0. ||
      result.max_wrist_roll_deviation>std::acos(-1.))
    throw std::invalid_argument("Invalid posture optimization settings");
  return result;
}
}  // namespace onero_a1_kinematics
