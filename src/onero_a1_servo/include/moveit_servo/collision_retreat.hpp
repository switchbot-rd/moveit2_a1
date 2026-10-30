// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <onero_a1_servo/moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{
// Ignore the wrist/gripper mounting clearance for proximity scaling only.
// Actual collision checks always use the scene's original collision matrix.
collision_detection::AllowedCollisionMatrix proximityCollisionMatrix(const planning_scene::PlanningScene& scene);

// Check every nearby world/self pair, including command latency and braking.
bool collisionRetreatSafe(const planning_scene::PlanningScene& scene, const KinematicState& current,
                          const KinematicState& candidate, const servo::Params& params);
bool collisionFreeCommand(const planning_scene::PlanningScene& scene, const KinematicState& command,
                          const servo::Params& params);
bool rebaseCollisionCommand(const planning_scene::PlanningScene& scene, KinematicState& command,
                            const servo::Params& params);
}  // namespace moveit_servo
