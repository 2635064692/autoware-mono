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

#ifndef AUTOWARE__FOLLOWING_GOAL_PUBLISHER__GOAL_GENERATOR_HPP_
#define AUTOWARE__FOLLOWING_GOAL_PUBLISHER__GOAL_GENERATOR_HPP_

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <optional>
#include <string>

namespace autoware::following_goal_publisher
{

constexpr double kDefaultAheadDistanceM = 50.0;

struct GoalPoseResult
{
  std::optional<geometry_msgs::msg::PoseStamped> goal;
  bool used_fallback{false};
  std::string debug;
};

GoalPoseResult generate_goal_pose_ahead(
  const autoware_perception_msgs::msg::PredictedObject & target,
  const std_msgs::msg::Header & header, double ahead_distance_m,
  const std::optional<geometry_msgs::msg::PoseStamped> & last_goal);

}  // namespace autoware::following_goal_publisher

#endif  // AUTOWARE__FOLLOWING_GOAL_PUBLISHER__GOAL_GENERATOR_HPP_
