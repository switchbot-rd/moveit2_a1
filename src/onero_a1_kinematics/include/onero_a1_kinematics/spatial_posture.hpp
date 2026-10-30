// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <onero_a1_kinematics/profile.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <algorithm>

namespace onero_a1_kinematics
{
// The same spatial objective is used by endpoint selection and both native IK
// solvers. Link4 is the elbow; the Link4--Link6 midpoint represents the forearm.
class SpatialPosture
{
  struct Term
  {
    const moveit::core::LinkModel* first;
    const moveit::core::LinkModel* second;
    const moveit::core::JointModelGroup* jacobian_group;
    Eigen::Vector3d base, outward;
    double preferred, scale, weight;
  };
  const moveit::core::JointModelGroup* group_;
  std::vector<Term> terms_;
public:
  SpatialPosture(const moveit::core::RobotState& seed,
                 const moveit::core::JointModelGroup* group,const Profile& profile) : group_(group)
  {
    if (!profile.enabled)return;
    auto reference=seed;
    reference.setJointGroupPositions(group,profile.reference);reference.update();
    const auto& model=*reference.getRobotModel();
    const auto& names=group->getVariableNames();
    for (const std::string side:{"l","r"})
    {
      const auto joint=std::find(names.begin(),names.end(),"joint3-a1_"+side);
      if (joint==names.end())continue;
      const std::string arm=side=="l"?"left":"right";
      const std::string prefix=model.hasLinkModel(arm+"_Link4")?arm+"_":"";
      const auto* elbow=model.getLinkModel(prefix+"Link4");
      const auto* wrist=model.getLinkModel(prefix+"Link6");
      if (!elbow || !wrist)continue;
      const Eigen::Vector3d base=reference.getGlobalLinkTransform(prefix+"base_link").translation();
      const Eigen::Vector3d shoulder=reference.getGlobalLinkTransform(prefix+"Link2").translation();
      const Eigen::Vector3d e=reference.getGlobalLinkTransform(elbow).translation();
      const Eigen::Vector3d w=reference.getGlobalLinkTransform(wrist).translation();
      Eigen::Vector3d outward=shoulder-base;
      const std::string other=(side=="l"?"right":"left")+std::string("_base_link");
      if (!prefix.empty() && model.hasLinkModel(other))
        outward=base-reference.getGlobalLinkTransform(other).translation();
      if (outward.norm()<1e-6)continue;
      outward.normalize();
      const double weight=4.*profile.weights[std::distance(names.begin(),joint)];
      const auto* jacobian_group=group->isChain()?group:model.getJointModelGroup(arm+"_arm");
      if (!jacobian_group)continue;
      terms_.push_back({elbow,nullptr,jacobian_group,base,outward,(e-base).dot(outward),
                        std::max(.1,(e-shoulder).norm()),weight});
      terms_.push_back({elbow,wrist,jacobian_group,base,outward,(.5*(e+w)-base).dot(outward),
                        std::max(.1,(w-e).norm()),weight});
    }
  }
  std::pair<double,Eigen::VectorXd> evaluate(const moveit::core::RobotState& state, bool with_gradient=true) const
  {
    double cost=0.;
    Eigen::VectorXd gradient=Eigen::VectorXd::Zero(group_->getVariableCount());
    for (const auto& term:terms_)
    {
      Eigen::Vector3d point=state.getGlobalLinkTransform(term.first).translation();
      if (term.second)point=.5*(point+state.getGlobalLinkTransform(term.second).translation());
      const double deficit=std::max(0.,term.preferred-(point-term.base).dot(term.outward));
      cost+=term.weight*deficit*deficit/(term.scale*term.scale);
      if (deficit<=0. || !with_gradient)continue;
      const auto* root=term.jacobian_group->getJointModels().front()->getParentLinkModel();
      const Eigen::Matrix3d rotation=state.getGlobalLinkTransform(root).rotation();
      Eigen::MatrixXd first,second;
      if (!state.getJacobian(term.jacobian_group,term.first,Eigen::Vector3d::Zero(),first))continue;
      Eigen::MatrixXd linear=first.topRows(3);
      if (term.second)
      {
        if (!state.getJacobian(term.jacobian_group,term.second,Eigen::Vector3d::Zero(),second))continue;
        linear=.5*(linear+second.topRows(3)).eval();
      }
      const Eigen::RowVectorXd derivative=term.outward.transpose()*rotation*linear;
      const auto& full_names=group_->getVariableNames();
      const auto& arm_names=term.jacobian_group->getVariableNames();
      for (std::size_t i=0;i<arm_names.size();++i)
      {
        const auto item=std::find(full_names.begin(),full_names.end(),arm_names[i]);
        if (item!=full_names.end())
          gradient[std::distance(full_names.begin(),item)]-=2.*term.weight*deficit/(term.scale*term.scale)*derivative[i];
      }
    }
    return {cost,gradient};
  }
};

inline Eigen::VectorXd postureVelocity(const Profile& profile,const Eigen::VectorXd& q,
                                      const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& extra_gradient)
{
  if (!profile.enabled)return Eigen::VectorXd::Zero(q.size());
  Eigen::VectorXd velocity=projectNullspace(jacobian,-profile.gain*(profile.gradient(q)+extra_gradient));
  velocity*=std::min(1.,profile.max_speed/std::max(1e-12,velocity.cwiseAbs().maxCoeff()));
  return velocity;
}
} // namespace onero_a1_kinematics
