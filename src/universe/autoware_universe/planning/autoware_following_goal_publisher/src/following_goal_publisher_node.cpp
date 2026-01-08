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

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  return tf2::getYaw(pose.orientation);
}

double normalize_angle_rad(const double rad)
{
  if (!std::isfinite(rad)) {
    return 0.0;
  }
  const double two_pi = 2.0 * M_PI;
  double v = std::fmod(rad + M_PI, two_pi);
  if (v < 0.0) {
    v += two_pi;
  }
  return v - M_PI;
}

double yaw_diff_abs_rad(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dyaw = normalize_angle_rad(yaw_from_pose(a) - yaw_from_pose(b));
  return std::fabs(dyaw);
}

double position_diff_xy_m(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  return std::hypot(dx, dy);
}

}  // namespace

FollowingGoalPublisherNode::FollowingGoalPublisherNode(const rclcpp::NodeOptions & options)
: Node("following_goal_publisher", options),
  tracker_(TargetTracker::Params{})
{
  enable_auto_follow_ = declare_parameter<bool>("enable_auto_follow", false);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
  ahead_distance_ = declare_parameter<double>("ahead_distance", kDefaultAheadDistanceM);

  // Keep parameter surface aligned with later throttle work (AFBUS50M-040).
  update_threshold_position_m_ = declare_parameter<double>("update_threshold_position_m", 2.0);
  update_threshold_yaw_deg_ = declare_parameter<double>("update_threshold_yaw_deg", 5.0);

  debug_throttle_ms_ = declare_parameter<int>("debug_throttle_ms", 2000);

  target_label_ = declare_parameter<std::string>("target_type", "BUS");
  lost_timeout_sec_ = declare_parameter<double>("lost_timeout_sec", 1.0);
  search_range_m_ =
    declare_parameter<std::vector<double>>("search_range_m", {0.0, 120.0});

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

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&FollowingGoalPublisherNode::on_parameters, this, std::placeholders::_1));

  if (!(std::isfinite(publish_rate_hz_) && publish_rate_hz_ > 0.0)) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz is invalid; fallback to 2.0Hz");
    publish_rate_hz_ = 2.0;
  }
  const auto period = rclcpp::Rate(publish_rate_hz_).period();
  timer_ = create_wall_timer(period, std::bind(&FollowingGoalPublisherNode::on_timer, this));
}

rcl_interfaces::msg::SetParametersResult FollowingGoalPublisherNode::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : parameters) {
    if (p.get_name() == "enable_auto_follow") {
      enable_auto_follow_ = p.as_bool();
      if (!enable_auto_follow_) {
        std::lock_guard<std::mutex> lock(mutex_);
        locked_uuid_.reset();
        last_goal_.reset();
      }
      RCLCPP_INFO(
        get_logger(), "enable_auto_follow=%s", enable_auto_follow_ ? "true" : "false");
    }
  }

  return result;
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
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_, "%s", tracker_result.debug.c_str());
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
      "No locked target (target_type=%s); skip publishing goal.", target_label_.c_str());
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
      "No subscribers on /planning/mission_planning/goal (routing_adaptor not running?); "
      "publishing anyway.");
  }

  const auto publisher_count = count_publishers("/planning/mission_planning/goal");
  if (publisher_count > 1U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "Multiple publishers detected on /planning/mission_planning/goal (count=%zu). "
      "In auto-follow mode, manual goal may be overridden; disable enable_auto_follow for "
      "manual goal.",
      publisher_count);
  }

  const auto goal_res =
    generate_goal_pose_ahead(*it, objects->header, ahead_distance_, last_goal);
  if (!goal_res.goal) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), debug_throttle_ms_,
      "Failed to generate goal pose (%s); skip publishing.", goal_res.debug.c_str());
    return;
  }

  if (last_goal) {
    const double pos_diff = position_diff_xy_m(goal_res.goal->pose, last_goal->pose);
    const double yaw_diff = yaw_diff_abs_rad(goal_res.goal->pose, last_goal->pose);
    const double yaw_threshold_rad = update_threshold_yaw_deg_ * M_PI / 180.0;
    if (pos_diff < update_threshold_position_m_ && yaw_diff < yaw_threshold_rad) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), debug_throttle_ms_,
        "Goal change below threshold (pos=%.3fm yaw=%.3frad); skip publishing.",
        pos_diff, yaw_diff);
      return;
    }
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
