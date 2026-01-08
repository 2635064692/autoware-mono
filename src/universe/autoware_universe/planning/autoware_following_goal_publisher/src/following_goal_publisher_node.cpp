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

#include "autoware/following_goal_publisher/following_goal_publisher_node.hpp"

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

namespace autoware::following_goal_publisher
{
namespace
{

rclcpp::QoS make_goal_qos()
{
  auto qos = rclcpp::QoS{5};
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  return qos;
}

}  // namespace

FollowingGoalPublisherNode::FollowingGoalPublisherNode(const rclcpp::NodeOptions & options)
: Node("following_goal_publisher", options),
  tracker_(TargetTracker::Params{})
{
  enable_auto_follow_ = declare_parameter<bool>("enable_auto_follow", false);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
  ahead_distance_m_ = declare_parameter<double>("ahead_distance_m", kDefaultAheadDistanceM);

  // Keep parameter surface aligned with later throttle work (AFBUS50M-040).
  update_threshold_position_m_ = declare_parameter<double>("update_threshold_position_m", 2.0);
  update_threshold_yaw_deg_ = declare_parameter<double>("update_threshold_yaw_deg", 5.0);

  debug_throttle_ms_ = declare_parameter<int>("debug_throttle_ms", 2000);

  target_label_ = declare_parameter<std::string>("target_type", "BUS");
  lost_timeout_sec_ = declare_parameter<double>("lost_timeout_sec", 1.0);
  search_range_m_ = declare_parameter<std::vector<double>>("search_range_m", {0.0, 120.0});

  // Keep tracker params consistent with declared parameters.
  TargetTracker::Params p;
  p.target_label = target_label_;
  if (search_range_m_.size() == 2) {
    p.min_longitudinal_distance = search_range_m_[0];
    p.max_longitudinal_distance = search_range_m_[1];
  }
  p.lost_timeout_sec = lost_timeout_sec_;
  tracker_ = TargetTracker(p);

  sub_odometry_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&FollowingGoalPublisherNode::on_odometry, this, std::placeholders::_1));
  sub_objects_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects", rclcpp::QoS{1},
    std::bind(&FollowingGoalPublisherNode::on_objects, this, std::placeholders::_1));

  pub_goal_ = create_publisher<PoseStamped>("/planning/mission_planning/goal", make_goal_qos());

  if (!(std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0)) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz is invalid; fallback to 2.0Hz");
    publish_rate_hz_ = 2.0;
  }
  const auto period = rclcpp::Rate(publish_rate_hz_).period();
  timer_ = create_wall_timer(period, std::bind(&FollowingGoalPublisherNode::on_timer, this));
}

void FollowingGoalPublisherNode::on_odometry(const Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odometry_ = msg;
}

void FollowingGoalPublisherNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_objects_ = msg;
    odom = latest_odometry_;
  }

  if (!odom) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "No odometry yet; cannot update following target lock.");
    return;
  }

  const auto tracker_result = tracker_.update(*msg, *odom);
  if (tracker_result.changed) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), debug_throttle_ms_, "%s", tracker_result.debug.c_str());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  locked_uuid_ = tracker_result.locked_uuid;
}

void FollowingGoalPublisherNode::on_timer()
{
  if (!enable_auto_follow_) {
    return;
  }

  PredictedObjects::ConstSharedPtr objects;
  Odometry::ConstSharedPtr odom;
  std::optional<unique_identifier_msgs::msg::UUID> locked;
  std::optional<PoseStamped> last_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    objects = latest_objects_;
    odom = latest_odometry_;
    locked = locked_uuid_;
    last_goal = last_goal_;
  }

  if (!objects || !odom) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "Missing input (objects/odometry); skip publishing goal.");
    return;
  }

  if (!locked) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "No locked BUS; skip publishing goal.");
    return;
  }

  const auto it = std::find_if(
    objects->objects.begin(), objects->objects.end(), [&locked](const PredictedObject & o) {
      return o.object_id == *locked;
    });
  if (it == objects->objects.end()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "Locked uuid not found in latest PredictedObjects; skip publishing goal.");
    return;
  }

  if (pub_goal_->get_subscription_count() == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "No subscribers on /planning/mission_planning/goal (routing_adaptor not running?); publishing anyway.");
  }

  const auto goal_res = generate_goal_pose_ahead(*it, objects->header, ahead_distance_m_, last_goal);
  if (!goal_res.goal) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "Failed to generate goal pose (%s); skip publishing.", goal_res.debug.c_str());
    return;
  }

  pub_goal_->publish(*goal_res.goal);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_goal_ = *goal_res.goal;
  }
}

}  // namespace autoware::following_goal_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::following_goal_publisher::FollowingGoalPublisherNode)
