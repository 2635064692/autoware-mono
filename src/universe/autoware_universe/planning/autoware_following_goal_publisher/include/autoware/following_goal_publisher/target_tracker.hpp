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

#ifndef AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_HPP_
#define AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <optional>
#include <string>

namespace autoware::following_goal_publisher
{

class TargetTracker
{
public:
  struct Params
  {
    std::string target_label{"BUS"};
    double min_longitudinal_distance{0.0};
    double max_longitudinal_distance{120.0};
    double lost_timeout_sec{1.0};
  };

  explicit TargetTracker(const Params & params);

  struct Result
  {
    std::optional<unique_identifier_msgs::msg::UUID> locked_uuid;
    bool changed{false};
    std::string debug;
  };

  static std::string to_string(const unique_identifier_msgs::msg::UUID & uuid);

  void reset();

  Result update(
    const autoware_perception_msgs::msg::PredictedObjects & objects,
    const nav_msgs::msg::Odometry & ego_odometry);

private:
  Params params_;

  std::optional<unique_identifier_msgs::msg::UUID> locked_uuid_;
  rclcpp::Time locked_last_seen_stamp_{0, 0, RCL_ROS_TIME};

  static bool is_bus(
    const autoware_perception_msgs::msg::PredictedObject & object, const std::string & target_label);
  static double compute_longitudinal_distance(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const nav_msgs::msg::Odometry & ego_odometry);

  Result select_new_target(
    const autoware_perception_msgs::msg::PredictedObjects & objects,
    const nav_msgs::msg::Odometry & ego_odometry);
};

}  // namespace autoware::following_goal_publisher

#endif  // AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_HPP_
