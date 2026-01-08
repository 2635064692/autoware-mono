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

#include "autoware/following_goal_publisher/target_tracker.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace autoware::following_goal_publisher
{
namespace
{

double normalize_yaw(const double yaw)
{
  // Keep values finite and within [-pi, pi] for predictable dot products.
  if (!std::isfinite(yaw)) {
    return 0.0;
  }
  const double two_pi = 2.0 * M_PI;
  double v = std::fmod(yaw + M_PI, two_pi);
  if (v < 0.0) {
    v += two_pi;
  }
  return v - M_PI;
}

}  // namespace

TargetTracker::TargetTracker(const Params & params)
: params_(params) {}

void TargetTracker::reset()
{
  locked_uuid_.reset();
  locked_last_seen_stamp_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
}

TargetTracker::Result TargetTracker::update(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const nav_msgs::msg::Odometry & ego_odometry)
{
  Result result;

  const auto target_label_id = resolve_target_label_id(params_.target_label);
  if (!target_label_id) {
    locked_uuid_.reset();
    locked_last_seen_stamp_ = rclcpp::Time{0, 0, RCL_ROS_TIME};
    result.changed = true;
    result.debug = "unsupported target_label=" + params_.target_label;
    return result;
  }

  if (objects.header.frame_id.empty() || ego_odometry.header.frame_id.empty()) {
    result.debug = "missing frame_id";
    return result;
  }
  if (objects.header.frame_id != ego_odometry.header.frame_id) {
    result.debug =
      "frame mismatch objects=" + objects.header.frame_id + " ego=" + ego_odometry.header.frame_id;
    return result;
  }

  const rclcpp::Time now{objects.header.stamp};

  if (locked_uuid_) {
    const auto it = std::find_if(
      objects.objects.begin(), objects.objects.end(),
      [this](const auto & o) {return o.object_id == *locked_uuid_;});

    if (it != objects.objects.end()) {
      const double lon = compute_longitudinal_distance(*it, ego_odometry);
      const bool ahead = lon > 0.0;
      const bool in_range =
        params_.min_longitudinal_distance <= lon && lon <= params_.max_longitudinal_distance;
      const bool matches = matches_target(*it, *target_label_id);

      if (matches && ahead && in_range) {
        locked_last_seen_stamp_ = now;
        result.locked_uuid = locked_uuid_;
        result.debug =
          "keep locked_uuid=" + to_string(*locked_uuid_) + " lon=" + std::to_string(lon);
        return result;
      }

      result.changed = true;
      result.debug =
        "unlock invalid locked_uuid=" + to_string(*locked_uuid_) + " lon=" + std::to_string(lon);
      locked_uuid_.reset();
    } else {
      const double dt = (now - locked_last_seen_stamp_).seconds();
      if (0.0 <= locked_last_seen_stamp_.seconds() && dt <= params_.lost_timeout_sec) {
        result.locked_uuid = locked_uuid_;
        result.debug = "keep (temporarily lost) locked_uuid=" + to_string(*locked_uuid_) +
          " dt=" + std::to_string(dt);
        return result;
      }
      result.changed = true;
      result.debug =
        "unlock lost locked_uuid=" + to_string(*locked_uuid_) + " dt=" + std::to_string(dt);
      locked_uuid_.reset();
    }
  }

  const auto selected = select_new_target(objects, ego_odometry);
  if (selected.locked_uuid) {
    locked_uuid_ = selected.locked_uuid;
    locked_last_seen_stamp_ = now;
    return selected;
  }

  result.debug = locked_uuid_ ? ("locked_uuid=" + to_string(*locked_uuid_)) : "no target";
  result.locked_uuid = locked_uuid_;
  return result;
}

TargetTracker::Result TargetTracker::select_new_target(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const nav_msgs::msg::Odometry & ego_odometry)
{
  Result result;

  struct Candidate
  {
    unique_identifier_msgs::msg::UUID uuid;
    double longitudinal_distance;
  };

  const auto target_label_id = resolve_target_label_id(params_.target_label);
  if (!target_label_id) {
    result.changed = true;
    result.debug = "unsupported target_label=" + params_.target_label;
    return result;
  }

  std::optional<Candidate> best;
  for (const auto & o : objects.objects) {
    if (!matches_target(o, *target_label_id)) {
      continue;
    }
    const double lon = compute_longitudinal_distance(o, ego_odometry);
    if (!(lon > 0.0)) {
      continue;
    }
    if (!(params_.min_longitudinal_distance <= lon && lon <= params_.max_longitudinal_distance)) {
      continue;
    }
    if (!best || lon < best->longitudinal_distance) {
      best = Candidate{o.object_id, lon};
    }
  }

  if (!best) {
    result.debug = "no target";
    return result;
  }

  result.locked_uuid = best->uuid;
  result.changed = true;
  result.debug = "select locked_uuid=" + to_string(best->uuid) + " lon=" +
    std::to_string(best->longitudinal_distance);
  return result;
}

std::optional<uint8_t> TargetTracker::resolve_target_label_id(const std::string & target_label)
{
  using autoware_perception_msgs::msg::ObjectClassification;

  std::string normalized = target_label;
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char c) {return static_cast<char>(std::toupper(c));});

  if (normalized == "BUS") {
    return ObjectClassification::BUS;
  }
  if (normalized == "PEDESTRIAN" || normalized == "PERSON" || normalized == "HUMAN") {
    return ObjectClassification::PEDESTRIAN;
  }
  return std::nullopt;
}

bool TargetTracker::matches_target(
  const autoware_perception_msgs::msg::PredictedObject & object, const uint8_t target_label_id)
{
  return std::any_of(
    object.classification.begin(), object.classification.end(),
    [target_label_id](const auto & c) {return c.label == target_label_id;});
}

bool TargetTracker::matches_target_label(
  const autoware_perception_msgs::msg::PredictedObject & object, const std::string & target_label)
{
  const auto target_label_id = resolve_target_label_id(target_label);
  if (!target_label_id) {
    return false;
  }
  return matches_target(object, *target_label_id);
}

double TargetTracker::compute_longitudinal_distance(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const nav_msgs::msg::Odometry & ego_odometry)
{
  const auto & ego_p = ego_odometry.pose.pose.position;
  const auto & obj_p = object.kinematics.initial_pose_with_covariance.pose.position;

  const auto & q = ego_odometry.pose.pose.orientation;
  const double yaw = normalize_yaw(tf2::getYaw(q));
  const double fx = std::cos(yaw);
  const double fy = std::sin(yaw);

  const double rx = obj_p.x - ego_p.x;
  const double ry = obj_p.y - ego_p.y;
  return rx * fx + ry * fy;
}

std::string TargetTracker::to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream ss;
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; i < uuid.uuid.size(); ++i) {
    ss << std::setw(2) << static_cast<int>(uuid.uuid[i]);
  }
  return ss.str();
}

}  // namespace autoware::following_goal_publisher
