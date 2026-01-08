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

#ifndef AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_NODE_HPP_
#define AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_NODE_HPP_

#include "autoware/following_goal_publisher/target_tracker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>

namespace autoware::following_goal_publisher
{

class TargetTrackerNode : public rclcpp::Node
{
public:
  explicit TargetTrackerNode(const rclcpp::NodeOptions & options);

private:
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using Odometry = nav_msgs::msg::Odometry;
  using StringStamped = autoware_internal_debug_msgs::msg::StringStamped;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_objects(const PredictedObjects::ConstSharedPtr msg);

  std::mutex mutex_;
  Odometry::ConstSharedPtr latest_odometry_;
  TargetTracker tracker_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_locked_uuid_;

  int debug_throttle_ms_{2000};
  rclcpp::Time last_debug_pub_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace autoware::following_goal_publisher

#endif  // AUTOWARE__FOLLOWING_GOAL_PUBLISHER__TARGET_TRACKER_NODE_HPP_
