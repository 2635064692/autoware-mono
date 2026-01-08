// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/following_goal_publisher/goal_generator.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace autoware::following_goal_publisher
{
namespace
{

bool is_valid_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm2) && (1e-6 < norm2);
}

bool is_finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const bool pos_ok = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  const bool ori_ok = std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
  return pos_ok && ori_ok;
}

}  // namespace

GoalPoseResult generate_goal_pose_ahead(
  const autoware_perception_msgs::msg::PredictedObject & target,
  const std_msgs::msg::Header & header, const double ahead_distance_m,
  const std::optional<geometry_msgs::msg::PoseStamped> & last_goal)
{
  GoalPoseResult out;

  if (!(std::isfinite(ahead_distance_m) && ahead_distance_m > 0.0)) {
    out.debug = "invalid ahead_distance_m";
    return out;
  }

  const auto & base_pose = target.kinematics.initial_pose_with_covariance.pose;
  if (!is_finite_pose(base_pose)) {
    out.debug = "target pose contains NaN/inf";
    return out;
  }

  if (!is_valid_quaternion(base_pose.orientation)) {
    if (last_goal) {
      geometry_msgs::msg::PoseStamped goal = *last_goal;
      goal.header = header;
      out.goal = goal;
      out.used_fallback = true;
      out.debug = "fallback to last_goal (invalid target orientation)";
      return out;
    }
    out.debug = "invalid target orientation and no last_goal";
    return out;
  }

  const double yaw = tf2::getYaw(base_pose.orientation);
  if (!std::isfinite(yaw)) {
    if (last_goal) {
      geometry_msgs::msg::PoseStamped goal = *last_goal;
      goal.header = header;
      out.goal = goal;
      out.used_fallback = true;
      out.debug = "fallback to last_goal (non-finite yaw)";
      return out;
    }
    out.debug = "non-finite yaw and no last_goal";
    return out;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header = header;
  goal.pose = base_pose;
  goal.pose.position.x += ahead_distance_m * std::cos(yaw);
  goal.pose.position.y += ahead_distance_m * std::sin(yaw);

  if (!is_finite_pose(goal.pose)) {
    out.debug = "generated goal pose contains NaN/inf";
    return out;
  }

  out.goal = goal;
  out.debug = "ok";
  return out;
}

}  // namespace autoware::following_goal_publisher

