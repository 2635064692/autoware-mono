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

#include "autoware/following_goal_publisher/target_tracker_node.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>

namespace autoware::following_goal_publisher
{

TargetTrackerNode::TargetTrackerNode(const rclcpp::NodeOptions & options)
: Node("following_target_tracker", options),
  tracker_(
    TargetTracker::Params{
      declare_parameter<std::string>("target_label", "BUS"),
      declare_parameter<double>("min_longitudinal_distance", 0.0),
      declare_parameter<double>("max_longitudinal_distance", 120.0),
      declare_parameter<double>("lost_timeout_sec", 1.0)})
{
  debug_throttle_ms_ = declare_parameter<int>("debug_throttle_ms", 2000);

  sub_odometry_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&TargetTrackerNode::on_odometry, this, std::placeholders::_1));
  sub_objects_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS{1},
    std::bind(&TargetTrackerNode::on_objects, this, std::placeholders::_1));

  pub_locked_uuid_ =
    create_publisher<StringStamped>("~/debug/locked_target_uuid", rclcpp::QoS{1});
}

void TargetTrackerNode::on_odometry(const Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odometry_ = msg;
}

void TargetTrackerNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom = latest_odometry_;
  }

  if (!odom) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "No odometry yet; skip selecting following target.");
    return;
  }

  const auto result = tracker_.update(*msg, *odom);

  if (result.changed) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_, "%s", result.debug.c_str());
  } else {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_, "%s", result.debug.c_str());
  }

  const auto now_stamp = now();
  const auto should_publish_debug =
    result.changed ||
    ((now_stamp - last_debug_pub_stamp_).nanoseconds() >= (debug_throttle_ms_ * 1000LL * 1000LL));
  if (should_publish_debug) {
    last_debug_pub_stamp_ = now_stamp;
    StringStamped out;
    out.stamp = now_stamp;
    out.data = result.locked_uuid ? ("uuid=" + TargetTracker::to_string(*result.locked_uuid)) :
      "uuid=UNSET";
    pub_locked_uuid_->publish(out);
  }
}

}  // namespace autoware::following_goal_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::following_goal_publisher::TargetTrackerNode)
