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

#ifndef AUTOWARE__FOLLOWING_GOAL_PUBLISHER__FOLLOWING_GOAL_PUBLISHER_NODE_HPP_
#define AUTOWARE__FOLLOWING_GOAL_PUBLISHER__FOLLOWING_GOAL_PUBLISHER_NODE_HPP_

#include "autoware/following_goal_publisher/goal_generator.hpp"
#include "autoware/following_goal_publisher/target_tracker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <mutex>
#include <optional>

namespace autoware::following_goal_publisher
{

class FollowingGoalPublisherNode : public rclcpp::Node
{
public:
  explicit FollowingGoalPublisherNode(const rclcpp::NodeOptions & options);

private:
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
  using Odometry = nav_msgs::msg::Odometry;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_objects(const PredictedObjects::ConstSharedPtr msg);
  void on_timer();
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters);

  std::mutex mutex_;
  Odometry::ConstSharedPtr latest_odometry_;
  PredictedObjects::ConstSharedPtr latest_objects_;

  TargetTracker tracker_;
  std::optional<unique_identifier_msgs::msg::UUID> locked_uuid_;
  std::optional<PoseStamped> last_goal_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  bool enable_auto_follow_{false};
  double publish_rate_hz_{2.0};
  double ahead_distance_m_{kDefaultAheadDistanceM};

  // Declared here for forward compatibility (used in AFBUS50M-040).
  double update_threshold_position_m_{2.0};
  double update_threshold_yaw_deg_{5.0};

  double lost_timeout_sec_{1.0};
  std::vector<double> search_range_m_{0.0, 120.0};
  std::string target_label_{"BUS"};

  int debug_throttle_ms_{2000};
};

}  // namespace autoware::following_goal_publisher

#endif  // AUTOWARE__FOLLOWING_GOAL_PUBLISHER__FOLLOWING_GOAL_PUBLISHER_NODE_HPP_
