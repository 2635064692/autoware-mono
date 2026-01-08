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

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

namespace autoware::following_goal_publisher::testing
{
using autoware_perception_msgs::msg::PredictedObject;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Header;

Header make_header(const rclcpp::Time & stamp)
{
  Header h;
  h.frame_id = "map";
  h.stamp = stamp;
  return h;
}

PredictedObject make_target(const double x, const double y, const double yaw_rad)
{
  PredictedObject o;
  o.kinematics.initial_pose_with_covariance.pose.position.x = x;
  o.kinematics.initial_pose_with_covariance.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  o.kinematics.initial_pose_with_covariance.pose.orientation = tf2::toMsg(q);
  return o;
}

TEST(GoalGeneratorTest, StraightAheadDistanceIsAccurate)
{
  rclcpp::init(0, nullptr);

  const auto header = make_header(rclcpp::Time(10, 0, RCL_ROS_TIME));
  const auto target = make_target(1.0, 2.0, 0.0);
  const auto res = generate_goal_pose_ahead(target, header, 50.0, std::nullopt);

  ASSERT_TRUE(res.goal.has_value());
  EXPECT_NEAR(res.goal->pose.position.x, 51.0, 1e-6);
  EXPECT_NEAR(res.goal->pose.position.y, 2.0, 1e-6);
  EXPECT_EQ(res.goal->header.frame_id, "map");

  rclcpp::shutdown();
}

TEST(GoalGeneratorTest, FallbackToLastGoalWhenOrientationInvalid)
{
  rclcpp::init(0, nullptr);

  const auto header = make_header(rclcpp::Time(10, 0, RCL_ROS_TIME));
  PredictedObject target;
  target.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  target.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  target.kinematics.initial_pose_with_covariance.pose.orientation.w = 0.0;

  PoseStamped last;
  last.header = header;
  last.pose.position.x = 123.0;
  last.pose.position.y = 456.0;
  last.pose.orientation.w = 1.0;

  const auto res = generate_goal_pose_ahead(target, header, 50.0, last);
  ASSERT_TRUE(res.goal.has_value());
  EXPECT_TRUE(res.used_fallback);
  EXPECT_NEAR(res.goal->pose.position.x, 123.0, 1e-6);
  EXPECT_NEAR(res.goal->pose.position.y, 456.0, 1e-6);

  rclcpp::shutdown();
}

}  // namespace autoware::following_goal_publisher::testing

