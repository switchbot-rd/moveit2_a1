// SPDX-License-Identifier: BSD-3-Clause
#include <moveit_servo/collision_retreat.hpp>
#include <moveit_servo/utils/common.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_set>

namespace moveit_servo
{
namespace
{
constexpr double DISTANCE_TOLERANCE = 1e-6;  // 1 micrometre, numerical tolerance only.
constexpr double MAX_JOINT_STEP = 0.005;    // Sample at most 0.29 degrees apart.
using Body = std::pair<int, std::string>;
using Pair = std::pair<Body, Body>;
using Distances = std::map<Pair, double>;

bool collectDistances(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state,
                      const servo::Params& params, Distances& distances,
                      const collision_detection::AllowedCollisionMatrix& path_acm)
{
  state.updateCollisionBodyTransforms();
  for (const bool self : { false, true })
  {
    collision_detection::DistanceRequest request;
    request.group_name = params.move_group_name;
    request.enableGroup(scene.getRobotModel());
    request.acm = &scene.getAllowedCollisionMatrix();
    request.type = collision_detection::DistanceRequestType::SINGLE;
    request.enable_signed_distance = true;
    request.distance_threshold = self ? params.self_collision_proximity_threshold :
                                        params.scene_collision_proximity_threshold;
    collision_detection::DistanceResult result;
    if (self)
    {
      // Check actual intersections even for mounting pairs omitted from the
      // proximity matrix, at every predicted sample as well as measured joints.
      collision_detection::CollisionRequest collision_request;
      collision_request.group_name = params.move_group_name;
      collision_detection::CollisionResult collision_result;
      scene.getCollisionEnvUnpadded()->checkSelfCollision(collision_request, collision_result, state,
                                                        scene.getAllowedCollisionMatrix());
      if (collision_result.collision)
        return false;
    }
    if (self)
      request.acm = &path_acm;
    if (self)
      scene.getCollisionEnvUnpadded()->distanceSelf(request, result, state);
    else
      scene.getCollisionEnv()->distanceRobot(request, result, state);
    if (result.collision || result.minimum_distance.distance <= 0.)
      return false;
    for (const auto& [pair, data] : result.distances)
    {
      for (const auto& entry : data)
      {
        if (!std::isfinite(entry.distance) || entry.distance <= 0.)
          return false;
        if (entry.distance >= request.distance_threshold)
          continue;
        const Body first{ int(entry.body_types[0]), entry.link_names[0] };
        const Body second{ int(entry.body_types[1]), entry.link_names[1] };
        const Pair key = std::minmax(first, second);
        if (key.first.second.empty() || key.second.second.empty())
          return false;
        auto [position, inserted] = distances.emplace(key, entry.distance);
        if (!inserted)
          position->second = std::min(position->second, entry.distance);
      }
    }
  }
  return true;
}

bool preservesClearance(const Distances& baseline, const Distances& sample)
{
  for (const auto& [pair, distance] : sample)
  {
    const auto before = baseline.find(pair);
    // A previously clear pair must not enter a new proximity zone either.
    if (before == baseline.end() || distance + DISTANCE_TOLERANCE < before->second)
      return false;
  }
  return true;
}
}  // namespace

collision_detection::AllowedCollisionMatrix proximityCollisionMatrix(const planning_scene::PlanningScene& scene)
{
  collision_detection::AllowedCollisionMatrix matrix(scene.getAllowedCollisionMatrix());
  const auto& model = scene.getRobotModel();
  for (const std::string prefix : { "", "left_", "right_" })
  {
    const std::string wrist = prefix + "Link6", gripper = prefix + "woan_picker_base_link";
    if (model->hasLinkModel(wrist) && model->hasLinkModel(gripper))
      matrix.setEntry(wrist, gripper, true);
  }
  return matrix;
}

bool collisionFreeCommand(const planning_scene::PlanningScene& scene, const KinematicState& command,
                          const servo::Params& params)
{
  if (!command.positions.allFinite())
    return false;
  moveit::core::RobotState state(scene.getCurrentState());
  const auto* group = state.getJointModelGroup(params.move_group_name);
  state.setJointGroupPositions(group, command.positions);
  state.updateCollisionBodyTransforms();
  collision_detection::CollisionRequest request;
  request.group_name = params.move_group_name;
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state);
  return !result.collision && state.satisfiesBounds(group);
}

bool rebaseCollisionCommand(const planning_scene::PlanningScene& scene, KinematicState& command,
                            const servo::Params& params)
{
  if (collisionFreeCommand(scene, command, params))
    return false;
  auto measured = extractRobotState(std::make_shared<moveit::core::RobotState>(scene.getCurrentState()),
                                    params.move_group_name);
  if (!collisionFreeCommand(scene, measured, params))
    return false;
  measured.velocities.setZero();
  measured.accelerations.setZero();
  command = measured;
  return true;
}

bool collisionRetreatSafe(const planning_scene::PlanningScene& scene, const KinematicState& current,
                          const KinematicState& candidate, const servo::Params& params)
{
  const Eigen::VectorXd velocity = (candidate.positions - current.positions) / params.publish_period;
  if (!velocity.allFinite() || velocity.cwiseAbs().maxCoeff() < 1e-8 ||
      velocity.cwiseAbs().maxCoeff() > params.collision_retreat_joint_speed + 1e-6)
    return false;

  moveit::core::RobotState state(scene.getCurrentState());
  const auto* group = state.getJointModelGroup(params.move_group_name);
  auto path_acm = proximityCollisionMatrix(scene);
  const auto& arm_joints = group->getActiveJointModels();
  const std::unordered_set<const moveit::core::JointModel*> active(arm_joints.begin(), arm_joints.end());
  std::map<const moveit::core::JointModel*, std::vector<std::string>> shared_motion;
  for (const auto* link : scene.getRobotModel()->getLinkModelsWithCollisionGeometry())
  {
    const auto* ancestor = link;
    const moveit::core::JointModel* driver = nullptr;
    while (ancestor && ancestor->getParentJointModel())
    {
      const auto* joint = ancestor->getParentJointModel();
      if (active.count(joint))
      {
        driver = joint;
        break;
      }
      ancestor = ancestor->getParentLinkModel();
    }
    shared_motion[driver].push_back(link->getName());
  }
  // Links driven by the same last arm joint retain their relative pose during
  // arm-only prediction; keep their complete collision check, without repeated
  // proximity distance queries within each rigid assembly.
  for (const auto& [driver, links] : shared_motion)
    for (std::size_t first = 0; first < links.size(); ++first)
      for (std::size_t second = first + 1; second < links.size(); ++second)
        path_acm.setEntry(links[first], links[second], true);
  Distances measured;
  if (!collectDistances(scene, state, params, measured, path_acm))
    return false;
  Eigen::VectorXd actual;
  state.copyJointGroupPositions(group, actual);
  state.setJointGroupPositions(group, current.positions);
  Distances baseline;
  if (!collectDistances(scene, state, params, baseline, path_acm) || baseline.empty())
    return false;

  // The measured arm can trail the already issued command. Check that bridge
  // segment as well, without allowing less clearance than either endpoint.
  Distances bridge_floor = measured;
  for (const auto& [pair, distance] : baseline)
  {
    auto [entry, inserted] = bridge_floor.emplace(pair, distance);
    if (!inserted)
      entry->second = std::min(entry->second, distance);
  }
  const int bridge_steps = std::max(1, int(std::ceil((current.positions - actual).cwiseAbs().maxCoeff() / MAX_JOINT_STEP)));
  if (bridge_steps > 40)
    return false;
  for (int step = 1; step <= bridge_steps; ++step)
  {
    state.setJointGroupPositions(group, actual + (current.positions - actual) * (double(step) / bridge_steps));
    Distances sample;
    if (!collectDistances(scene, state, params, sample, path_acm) || !preservesClearance(bridge_floor, sample))
      return false;
  }

  Eigen::VectorXd acceleration(velocity.size());
  const auto& bounds = group->getActiveJointModelsBounds();
  for (Eigen::Index index = 0; index < velocity.size(); ++index)
  {
    const auto& bound = bounds[index]->front();
    if (!bound.acceleration_bounded_)
      return false;
    acceleration[index] = std::min(bound.max_acceleration_, -bound.min_acceleration_);
    if (!(acceleration[index] > 0.))
      return false;
  }
  const double lead = params.max_expected_latency + params.publish_period;
  const double duration = lead + (velocity.cwiseAbs().cwiseQuotient(acceleration)).maxCoeff();
  const int steps = std::max(2, int(std::ceil(duration * velocity.cwiseAbs().maxCoeff() / MAX_JOINT_STEP)));
  if (steps > 40)
    return false;
  Distances last;
  for (int step = 1; step <= steps; ++step)
  {
    const double time = duration * step / steps;
    Eigen::VectorXd position = current.positions;
    for (Eigen::Index index = 0; index < velocity.size(); ++index)
    {
      const double braking_time = std::min(std::max(0., time - lead), std::abs(velocity[index]) / acceleration[index]);
      position[index] += velocity[index] * std::min(time, lead) +
                         std::copysign(std::abs(velocity[index]) * braking_time -
                                           .5 * acceleration[index] * braking_time * braking_time, velocity[index]);
    }
    state.setJointGroupPositions(group, position);
    if (!state.satisfiesBounds(group))
      return false;
    last.clear();
    if (!collectDistances(scene, state, params, last, path_acm) || !preservesClearance(baseline, last))
      return false;
  }
  // At least one proximity pair must improve; tangential motion is not escape.
  return std::any_of(baseline.begin(), baseline.end(), [&](const auto& entry) {
    const auto end = last.find(entry.first);
    return end == last.end() || end->second > entry.second + DISTANCE_TOLERANCE;
  });
}
}  // namespace moveit_servo
