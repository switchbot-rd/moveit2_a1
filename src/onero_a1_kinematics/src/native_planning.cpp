// SPDX-License-Identifier: Apache-2.0
#include <onero_a1_kinematics/profile.hpp>
#include <onero_a1_kinematics/spatial_posture.hpp>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <cstring>
#include <unordered_map>
#include <limits>

namespace py=pybind11;
namespace
{
template<class Message> Message decode(const py::bytes& input)
{
  const std::string bytes=input;
  rclcpp::SerializedMessage buffer(bytes.size());
  auto& storage=buffer.get_rcl_serialized_message();
  std::memcpy(storage.buffer,bytes.data(),bytes.size());storage.buffer_length=bytes.size();
  Message result;rclcpp::Serialization<Message> serializer;
  serializer.deserialize_message(&buffer,&result);return result;
}
template<class Message> py::bytes encode(const Message& message)
{
  rclcpp::SerializedMessage buffer;rclcpp::Serialization<Message> serializer;
  serializer.serialize_message(&message,&buffer);
  const auto& storage=buffer.get_rcl_serialized_message();
  return py::bytes(reinterpret_cast<const char*>(storage.buffer),storage.buffer_length);
}
class PlanningGeometry
{
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  moveit::core::RobotState state(const std::map<std::string,double>& positions) const
  {
    auto result=scene_->getCurrentState();
    result.setVariablePositions(positions);result.update();return result;
  }
public:
  PlanningGeometry(const std::string& urdf_text,const std::string& srdf_text)
  {
    // rclpy and rclcpp have separate contexts. Initialize the latter before
    // MoveIt creates its logging node; leave signal handling to the Python CLI.
    if (!rclcpp::ok())
      rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto urdf=urdf::parseURDF(urdf_text);
    if (!urdf)throw std::invalid_argument("Invalid robot URDF");
    auto srdf=std::make_shared<srdf::Model>();
    if (!srdf->initString(*urdf,srdf_text))throw std::invalid_argument("Invalid robot SRDF");
    model_=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
    scene_=std::make_shared<planning_scene::PlanningScene>(model_);
  }
  void setScene(const py::bytes& message)
  {
    if (!scene_->usePlanningSceneMsg(decode<moveit_msgs::msg::PlanningScene>(message)))
      throw std::runtime_error("Cannot restore MoveIt planning scene");
  }
  Eigen::Matrix4d fk(const std::map<std::string,double>& positions,const std::string& link,
                     const std::string& frame) const
  {
    auto q=state(positions);
    return (scene_->getFrameTransform(q,frame).inverse()*q.getGlobalLinkTransform(link)).matrix();
  }
  std::pair<bool,std::vector<std::pair<std::string,std::string>>> validity(
      const std::map<std::string,double>& positions,const std::string& group, const py::bytes& constraints) const
  {
    auto q=state(positions);
    std::vector<std::pair<std::string,std::string>> pairs;
    const std::string constraint_bytes=constraints;
    const bool valid=constraint_bytes.empty()?scene_->isStateValid(q,group):
        scene_->isStateValid(q,decode<moveit_msgs::msg::Constraints>(constraints),group);
    if (!valid)
    {
      collision_detection::CollisionRequest request;request.group_name=group;
      request.contacts=true;request.max_contacts=20;collision_detection::CollisionResult result;
      scene_->checkCollision(request,result,q);
      for (const auto& item:result.contacts)pairs.push_back(item.first);
    }
    return {valid,pairs};
  }
  std::map<std::string,double> taskIK(const std::map<std::string,double>& positions,
      const std::string& group,const std::string& link,const std::string& frame,
      const Eigen::Vector3d& point,const Eigen::Vector3d& offset, const Eigen::Vector3d& axis) const
  {
    const auto* joints=model_->getJointModelGroup(group);
    const auto* tip=model_->getLinkModel(link);
    if (!joints || !tip)throw std::invalid_argument("Unknown position IK group or link");
    const auto names=joints->getActiveJointModelNames();
    auto profile=onero_a1_kinematics::loadProfile(names);
    Eigen::VectorXd low(names.size()),high(names.size());
    for (std::size_t i=0;i<names.size();++i)
    { const auto& bound=model_->getVariableBounds(names[i]);low[i]=bound.min_position_;high[i]=bound.max_position_; }
    profile.setBounds(low,high);
    auto robot=state(positions);
    Eigen::VectorXd q;robot.copyJointGroupPositions(joints,q);
    const Eigen::VectorXd initial=q;
    onero_a1_kinematics::SpatialPosture spatial(robot,joints,profile);
    const auto reference=scene_->getFrameTransform(robot,frame);
    const Eigen::Vector3d target=reference*point;
    const bool upright=axis.norm()>.5;
    const Eigen::Vector3d up=reference.rotation().col(2);
    const auto* root=joints->getJointModels().front()->getParentLinkModel();
    bool found=false;double best=std::numeric_limits<double>::infinity();Eigen::VectorXd selected;
    int settled=0;
    for (int iteration=0;iteration<120;++iteration)
    {
      robot.setJointGroupPositions(joints,q);robot.update();
      Eigen::VectorXd error(upright?5:3);
      error.head<3>()=target-robot.getGlobalLinkTransform(tip)*offset;
      if (upright)
      {
        const Eigen::Vector3d actual=robot.getGlobalLinkTransform(tip).rotation()*axis.normalized();
        Eigen::Vector3d turn=actual.cross(up);
        const double angle=std::acos(std::clamp(actual.dot(up),-1.,1.));
        if (turn.norm()>1e-9)turn*=angle/turn.norm();
        else if (angle>1.)turn=angle*reference.rotation().col(0);
        error[3]=turn.dot(reference.rotation().col(0));
        error[4]=turn.dot(reference.rotation().col(1));
      }
      const auto spatial_state=spatial.evaluate(robot);
      if (error.head<3>().norm()<.0005 && (!upright || error.tail<2>().norm()<.005))
      {
        const double cost=profile.cost(q)+spatial_state.first+profile.continuity_weight*(q-initial).squaredNorm();
        if (robot.satisfiesBounds(joints) && scene_->isStateValid(robot,group) && cost<best)
        {selected=q;best=cost;found=true;}
        if (++settled>profile.refinement_steps)break;
      }
      Eigen::MatrixXd full;
      if (!robot.getJacobian(joints,tip,offset,full))break;
      const Eigen::Matrix3d rotate=robot.getGlobalLinkTransform(root).rotation();
      Eigen::MatrixXd jacobian(upright?5:3,full.cols());
      jacobian.topRows(3)=rotate*full.topRows(3);
      if (upright)
      {
        const Eigen::MatrixXd angular=rotate*full.bottomRows(3);
        jacobian.row(3)=reference.rotation().col(0).transpose()*angular;
        jacobian.row(4)=reference.rotation().col(1).transpose()*angular;
      }
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
      const double ratio=std::min(1.,svd.singularValues().tail(1)[0]/.02);
      const double damping=1e-6+.01*(1.-ratio*ratio);
      const Eigen::MatrixXd identity=Eigen::MatrixXd::Identity(error.size(),error.size());
      const Eigen::MatrixXd pinv=jacobian.transpose()*(jacobian*jacobian.transpose()+
          damping*identity).ldlt().solve(identity);
      // Position-only: 3x7; upright bottle: 5x7, leaving yaw and redundancy free.
      Eigen::VectorXd delta=pinv*error+onero_a1_kinematics::postureVelocity(profile,q,jacobian,
          spatial_state.second+2.*profile.continuity_weight*(q-initial));
      delta*=std::min(1.,.2/std::max(1e-12,delta.cwiseAbs().maxCoeff()));
      const Eigen::VectorXd next=(q+delta).cwiseMax(low).cwiseMin(high);
      if ((next-q).norm()<1e-8)break;
      q=next;
    }
    if (!found)return {};
    auto result=positions;
    for (std::size_t i=0;i<names.size();++i)result[names[i]]=selected[i];
    return result;
  }
  std::pair<double,Eigen::VectorXd> posture(const std::map<std::string,double>& positions,
      const std::string& group) const
  {
    const auto* joints=model_->getJointModelGroup(group);
    if (!joints)throw std::invalid_argument("Unknown posture group");
    auto profile=onero_a1_kinematics::loadProfile(joints->getActiveJointModelNames());
    const auto& names=joints->getVariableNames();
    Eigen::VectorXd low(names.size()),high(names.size());
    for (std::size_t i=0;i<names.size();++i)
    { const auto& bound=model_->getVariableBounds(names[i]);low[i]=bound.min_position_;high[i]=bound.max_position_; }
    profile.setBounds(low,high);
    auto robot=state(positions);Eigen::VectorXd q;robot.copyJointGroupPositions(joints,q);
    if (!profile.enabled)return {0.,Eigen::VectorXd::Zero(q.size())};
    onero_a1_kinematics::SpatialPosture spatial(robot,joints,profile);
    const auto extra=spatial.evaluate(robot);
    return {profile.cost(q)+extra.first,profile.gradient(q)+extra.second};
  }
  std::map<std::string,double> motionPosture(const std::map<std::string,double>& positions,
      const std::string& group,const py::bytes& message) const
  {
    const auto* joints=model_->getJointModelGroup(group);
    if (!joints)throw std::invalid_argument("Unknown posture group");
    auto profile=onero_a1_kinematics::loadProfile(joints->getActiveJointModelNames());
    const auto& names=joints->getVariableNames();
    Eigen::VectorXd low(names.size()),high(names.size());
    for (std::size_t i=0;i<names.size();++i)
    { const auto& bound=model_->getVariableBounds(names[i]);low[i]=bound.min_position_;high[i]=bound.max_position_; }
    profile.setBounds(low,high);
    auto reference=state(positions);
    robot_trajectory::RobotTrajectory path(model_,group);
    path.setRobotTrajectoryMsg(reference,decode<moveit_msgs::msg::RobotTrajectory>(message));
    if (path.empty())throw std::invalid_argument("Empty posture path");
    onero_a1_kinematics::SpatialPosture spatial(reference,joints,profile);
    auto roll=[&](const Eigen::VectorXd& q) {
      double result=0.;
      for (Eigen::Index k=0;k<q.size();++k)
        if (k%7==4 || k%7==6)result=std::max(result,std::abs(q[k]-profile.reference[k]));
      return result;
    };
    Eigen::VectorXd previous;reference.copyJointGroupPositions(joints,previous);
    double previous_cost=0.,cost=0.,peak=roll(previous),end=peak;
    const double start=peak;
    for (std::size_t i=0;i<path.getWayPointCount();++i)
    {
      auto& robot=*path.getWayPointPtr(i);robot.update();
      Eigen::VectorXd q;robot.copyJointGroupPositions(joints,q);
      const double objective=profile.enabled?profile.cost(q)+spatial.evaluate(robot,false).first:0.;
      if (i)cost+=(q-previous).norm()*(1.+profile.path_weight*.5*(objective+previous_cost));
      end=roll(q);peak=std::max(peak,end);previous=q;previous_cost=objective;
    }
    return {{"cost",cost+previous_cost},{"start_roll",start},{"peak_roll",peak},{"end_roll",end}};
  }
  py::bytes retime(const std::string& group,const py::bytes& start,const std::vector<py::bytes>& paths,
                   const std::unordered_map<std::string,double>& velocity,
                   const std::unordered_map<std::string,double>& acceleration, double tolerance) const
  {
    if (paths.empty())throw std::invalid_argument("Continuous motion needs a path");
    moveit::core::RobotState reference(model_);reference.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(decode<moveit_msgs::msg::RobotState>(start),reference);
    robot_trajectory::RobotTrajectory result(model_,group);
    for (const auto& bytes:paths)
    {
      robot_trajectory::RobotTrajectory path(model_,group);
      path.setRobotTrajectoryMsg(reference,decode<moveit_msgs::msg::RobotTrajectory>(bytes));
      result.append(path,0.,result.empty()?0:1);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(tolerance,.05,.001);
    if (!timing.computeTimeStamps(result,velocity,acceleration))
      throw std::runtime_error("MoveIt could not time-parameterize the complete motion");
    moveit_msgs::msg::RobotTrajectory message;result.getRobotTrajectoryMsg(message);return encode(message);
  }
};
}
void bindNativePlanning(py::module_& module)
{
  py::class_<PlanningGeometry,std::shared_ptr<PlanningGeometry>>(module,"PlanningGeometry")
    .def(py::init<const std::string&,const std::string&>())
    .def("set_scene",&PlanningGeometry::setScene).def("fk",&PlanningGeometry::fk)
    .def("validity",&PlanningGeometry::validity,py::arg("positions"),py::arg("group"),py::arg("constraints")=py::bytes())
    .def("position_ik",[](const PlanningGeometry& self,const std::map<std::string,double>& positions,
        const std::string& group,const std::string& link,const std::string& frame,
        const Eigen::Vector3d& point,const Eigen::Vector3d& offset)
        {return self.taskIK(positions,group,link,frame,point,offset,Eigen::Vector3d::Zero());})
    .def("upright_ik",&PlanningGeometry::taskIK)
    .def("posture",&PlanningGeometry::posture)
    .def("motion_posture",&PlanningGeometry::motionPosture)
    .def("retime",&PlanningGeometry::retime,py::arg("group"),py::arg("start"),py::arg("paths"),
         py::arg("velocity"),py::arg("acceleration"),py::arg("tolerance")=.005);
}
