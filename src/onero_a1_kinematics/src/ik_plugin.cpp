// SPDX-License-Identifier: Apache-2.0
#include <onero_a1_kinematics/profile.hpp>
#include <onero_a1_kinematics/spatial_posture.hpp>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <chrono>
#include <algorithm>
#include <limits>
#include <Eigen/Cholesky>
#include <Eigen/SVD>

namespace onero_a1_kinematics
{
class NaturalPostureIK : public kdl_kinematics_plugin::KDLKinematicsPlugin
{
  using Base=kdl_kinematics_plugin::KDLKinematicsPlugin;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* group_=nullptr;
  Profile profile_;

  static Eigen::VectorXd vector(const std::vector<double>& values)
  { return Eigen::Map<const Eigen::VectorXd>(values.data(),values.size()); }
  static std::vector<double> values(const Eigen::VectorXd& q)
  { return {q.data(),q.data()+q.size()}; }

public:
  bool initialize(const rclcpp::Node::SharedPtr& node,const moveit::core::RobotModel& model,
                  const std::string& group,const std::string& base,const std::vector<std::string>& tips,
                  double discretization) override
  {
    if (!Base::initialize(node,model,group,base,tips,discretization)) return false;
    try
    {
      model_=moveit::core::RobotModelConstPtr(&model,[](const moveit::core::RobotModel*){});
      group_=model.getJointModelGroup(group);
      profile_=loadProfile(getJointNames());
      Eigen::VectorXd low(getJointNames().size()),high(low.size());
      for (Eigen::Index i=0;i<low.size();++i)
      {
        const auto& bound=model.getVariableBounds(getJointNames()[i]);
        low[i]=bound.min_position_;high[i]=bound.max_position_;
      }
      profile_.setBounds(low,high);
      return true;
    }
    catch (const std::exception& error)
    { RCLCPP_ERROR(node->get_logger(),"Natural posture IK configuration: %s",error.what());return false; }
  }

  bool getPositionIK(const geometry_msgs::msg::Pose& pose,const std::vector<double>& seed,
                     std::vector<double>& solution,moveit_msgs::msg::MoveItErrorCodes& error,
                     const kinematics::KinematicsQueryOptions& options) const override
  { return searchPositionIK(pose,seed,getDefaultTimeout(),{},solution,{},error,options); }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose,const std::vector<double>& seed,double timeout,
                        std::vector<double>& solution,moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions& options) const override
  { return searchPositionIK(pose,seed,timeout,{},solution,{},error,options); }
  bool searchPositionIK(const geometry_msgs::msg::Pose& pose,const std::vector<double>& seed,double timeout,
                        const std::vector<double>& consistency,std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions& options) const override
  { return searchPositionIK(pose,seed,timeout,consistency,solution,{},error,options); }
  bool searchPositionIK(const geometry_msgs::msg::Pose& pose,const std::vector<double>& seed,double timeout,
                        std::vector<double>& solution,const IKCallbackFn& callback,
                        moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions& options) const override
  { return searchPositionIK(pose,seed,timeout,{},solution,callback,error,options); }

  bool searchPositionIK(const geometry_msgs::msg::Pose& pose,const std::vector<double>& seed,double timeout,
                        const std::vector<double>& consistency,std::vector<double>& solution,
                        const IKCallbackFn& callback,moveit_msgs::msg::MoveItErrorCodes& error,
                        const kinematics::KinematicsQueryOptions& options) const override
  {
    if (!profile_.enabled || options.lock_redundant_joints)
      return Base::searchPositionIK(pose,seed,timeout,consistency,solution,callback,error,options);
    if (seed.size()!=getJointNames().size() || (!consistency.empty() && consistency.size()!=seed.size()))
    { error.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;return false; }
    using Clock=std::chrono::steady_clock;
    const auto deadline=Clock::now()+std::chrono::duration<double>(std::max(timeout,.001));
    auto remaining=[&](){ return std::chrono::duration<double>(deadline-Clock::now()).count(); };
    const Eigen::VectorXd initial=vector(seed);
    if (!initial.allFinite())
    { error.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;return false; }
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();state.setJointGroupPositions(group_,initial);state.update();
    const Eigen::Isometry3d current=state.getGlobalLinkTransform(getBaseFrame()).inverse()*
                                    state.getGlobalLinkTransform(getTipFrame());
    Eigen::Quaterniond target_rotation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    const Eigen::Vector3d target_position(pose.position.x,pose.position.y,pose.position.z);
    if (!target_position.allFinite() || !target_rotation.coeffs().allFinite() || target_rotation.norm()<1e-9)
    { error.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;return false; }
    const bool local=(current.translation()-target_position).norm()<.03 &&
                     Eigen::Quaterniond(current.rotation()).angularDistance(target_rotation)<.15;
    const double continuity=profile_.continuity_weight*(local?10.:1.);
    SpatialPosture spatial(state,group_,profile_);
    auto consistent=[&](const Eigen::VectorXd& q){
      if (consistency.empty())return true;
      if (consistency.size()!=seed.size())return false;
      for (std::size_t i=0;i<seed.size();++i)
        if (std::abs(q[i]-initial[i])>consistency[i]+1e-9)return false;
      return true;
    };
    // Match the SDK's task-priority iteration: primary DLS motion plus a
    // projected secondary posture step on EVERY iteration, from the warm seed.
    // This is not a few tiny adjustments applied after an arbitrary IK branch.
    const auto* tip=state.getLinkModel(getTipFrame());
    const auto* root=group_->getJointModels().front()->getParentLinkModel();
    const Eigen::Isometry3d base=state.getGlobalLinkTransform(getBaseFrame());
    Eigen::Isometry3d target=Eigen::Isometry3d::Identity();
    target.translation()=target_position;target.linear()=target_rotation.normalized().toRotationMatrix();
    std::vector<Eigen::VectorXd> seeds{initial};
    if (!local && consistency.empty())seeds.push_back(profile_.reference);
    bool found=false;double best=std::numeric_limits<double>::infinity();
    for (std::size_t seed_index=0;seed_index<seeds.size() && remaining()>0.;++seed_index)
    {
      Eigen::VectorXd q=seeds[seed_index].cwiseMax(profile_.lower).cwiseMin(profile_.upper);
      const auto phase_deadline=Clock::now()+std::chrono::duration<double>(
          .8*remaining()/static_cast<double>(seeds.size()-seed_index));
      int settled=0;
      for (int iteration=0;iteration<120 && Clock::now()<phase_deadline;++iteration)
      {
        state.setJointGroupPositions(group_,q);state.update();
        const Eigen::Isometry3d actual=base.inverse()*state.getGlobalLinkTransform(tip);
        Eigen::Matrix<double,6,1> err;
        err.head<3>()=target.translation()-actual.translation();
        const Eigen::AngleAxisd turn(target.rotation()*actual.rotation().transpose());
        err.tail<3>()=turn.angle()*turn.axis();
        const bool reached=err.head<3>().norm()<.0005 && err.tail<3>().norm()<.005;
        const auto spatial_state=spatial.evaluate(state);
        const double score=profile_.cost(q)+spatial_state.first+continuity*(q-initial).squaredNorm();
        if (reached && consistent(q))
        {
          auto candidate=values(q);
          moveit_msgs::msg::MoveItErrorCodes status;status.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          if (callback)callback(pose,candidate,status);
          if (status.val==moveit_msgs::msg::MoveItErrorCodes::SUCCESS && score<best)
          { solution=candidate;best=score;found=true; }
          // Cartesian samples must not keep rotating redundantly after the
          // requested pose is reached; that accumulated into visible forearm twist.
          if ((local && found) || ++settled>profile_.refinement_steps)break;
        }
        Eigen::MatrixXd jacobian;
        if (!state.getJacobian(group_,tip,Eigen::Vector3d::Zero(),jacobian))break;
        const Eigen::Matrix3d rotate=base.rotation().transpose()*state.getGlobalLinkTransform(root).rotation();
        jacobian.topRows(3)=rotate*jacobian.topRows(3).eval();
        jacobian.bottomRows(3)=rotate*jacobian.bottomRows(3).eval();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
        const double sigma=svd.singularValues().tail(1)[0];
        const double ratio=std::min(1.,sigma/.02);
        const double damping=1e-6+.01*(1.-ratio*ratio);
        const Eigen::MatrixXd pinv=jacobian.transpose()*(jacobian*jacobian.transpose()+
            damping*Eigen::MatrixXd::Identity(6,6)).ldlt().solve(Eigen::MatrixXd::Identity(6,6));
        const Eigen::VectorXd primary=pinv*err;
        Eigen::VectorXd secondary=postureVelocity(profile_,q,jacobian,
            spatial_state.second+2.*continuity*(q-initial));
        if (local)
        {
          const double cap=std::min(.01,.25*primary.cwiseAbs().maxCoeff());
          secondary*=std::min(1.,cap/std::max(1e-12,secondary.cwiseAbs().maxCoeff()));
        }
        Eigen::VectorXd delta=primary+secondary;
        delta*=std::min(1.,.2/std::max(1e-12,delta.cwiseAbs().maxCoeff()));
        Eigen::VectorXd next=(q+delta).cwiseMax(profile_.lower).cwiseMin(profile_.upper);
        if (!consistency.empty())
          for (std::size_t i=0;i<seed.size();++i)
            next[i]=std::clamp(next[i],initial[i]-consistency[i],initial[i]+consistency[i]);
        if ((next-q).norm()<1e-8)break;
        q=next;
      }
      if (local && found)break;
    }
    // Preserve the existing collision callback and consistency rules on fallback.
    if (!found && remaining()>0.)
      found=Base::searchPositionIK(pose,seed,remaining(),consistency,solution,callback,error,options);
    if (found)error.val=moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return found;
  }
};
}  // namespace onero_a1_kinematics

PLUGINLIB_EXPORT_CLASS(onero_a1_kinematics::NaturalPostureIK,kinematics::KinematicsBase)
