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

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <array>

namespace autoware::following_goal_publisher::testing
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

UUID make_uuid(const uint8_t seed)
{
  UUID id;
  for (size_t i = 0; i < id.uuid.size(); ++i) {
    id.uuid[i] = static_cast<uint8_t>(seed + i);
  }
  return id;
}

PredictedObject make_bus(const UUID & uuid, const double x, const double y)
{
  PredictedObject o;
  o.object_id = uuid;
  ObjectClassification c;
  c.label = ObjectClassification::BUS;
  c.probability = 1.0;
  o.classification.push_back(c);
  o.kinematics.initial_pose_with_covariance.pose.position.x = x;
  o.kinematics.initial_pose_with_covariance.pose.position.y = y;
  return o;
}

Odometry make_ego(const double x, const double y, const double yaw_rad)
{
  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  odom.pose.pose.orientation = tf2::toMsg(q);
  return odom;
}

PredictedObjects make_objects(const rclcpp::Time & stamp, const std::initializer_list<PredictedObject> & objects)
{
  PredictedObjects msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;
  msg.objects = objects;
  return msg;
}

class TargetTrackerTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TargetTrackerTest, SelectNearestFrontBus)
{
  TargetTracker::Params p;
  p.max_longitudinal_distance = 100.0;
  p.lost_timeout_sec = 1.0;
  TargetTracker tracker(p);

  const auto ego = make_ego(0.0, 0.0, 0.0);
  const auto id10 = make_uuid(10);
  const auto id20 = make_uuid(20);

  const auto t = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const auto objects = make_objects(t, {make_bus(id20, 20.0, 0.0), make_bus(id10, 10.0, 0.0)});
  const auto result = tracker.update(objects, ego);

  ASSERT_TRUE(result.locked_uuid.has_value());
  EXPECT_EQ(result.locked_uuid->uuid, id10.uuid);
  EXPECT_TRUE(result.changed);
}

TEST_F(TargetTrackerTest, KeepLockedUntilLostTimeout)
{
  TargetTracker::Params p;
  p.max_longitudinal_distance = 100.0;
  p.lost_timeout_sec = 2.0;
  TargetTracker tracker(p);

  const auto ego = make_ego(0.0, 0.0, 0.0);
  const auto id10 = make_uuid(10);
  const auto id20 = make_uuid(20);

  const auto t0 = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const auto objects0 = make_objects(t0, {make_bus(id10, 10.0, 0.0), make_bus(id20, 20.0, 0.0)});
  const auto r0 = tracker.update(objects0, ego);
  ASSERT_TRUE(r0.locked_uuid.has_value());
  EXPECT_EQ(r0.locked_uuid->uuid, id10.uuid);

  // id10 is temporarily lost but within timeout -> keep id10.
  const auto t1 = rclcpp::Time(11, 0, RCL_ROS_TIME);
  const auto objects1 = make_objects(t1, {make_bus(id20, 20.0, 0.0)});
  const auto r1 = tracker.update(objects1, ego);
  ASSERT_TRUE(r1.locked_uuid.has_value());
  EXPECT_EQ(r1.locked_uuid->uuid, id10.uuid);

  // After timeout -> unlock and select id20.
  const auto t2 = rclcpp::Time(13, 0, RCL_ROS_TIME);
  const auto objects2 = make_objects(t2, {make_bus(id20, 20.0, 0.0)});
  const auto r2 = tracker.update(objects2, ego);
  ASSERT_TRUE(r2.locked_uuid.has_value());
  EXPECT_EQ(r2.locked_uuid->uuid, id20.uuid);
}

TEST_F(TargetTrackerTest, ReSelectWhenLockedBecomesBehind)
{
  TargetTracker::Params p;
  p.max_longitudinal_distance = 100.0;
  p.lost_timeout_sec = 2.0;
  TargetTracker tracker(p);

  const auto ego = make_ego(0.0, 0.0, 0.0);
  const auto id10 = make_uuid(10);
  const auto id20 = make_uuid(20);

  const auto t0 = rclcpp::Time(10, 0, RCL_ROS_TIME);
  const auto objects0 = make_objects(t0, {make_bus(id10, 10.0, 0.0), make_bus(id20, 20.0, 0.0)});
  const auto r0 = tracker.update(objects0, ego);
  ASSERT_TRUE(r0.locked_uuid.has_value());
  EXPECT_EQ(r0.locked_uuid->uuid, id10.uuid);

  // id10 moved behind -> unlock and select id20.
  const auto t1 = rclcpp::Time(11, 0, RCL_ROS_TIME);
  const auto objects1 = make_objects(t1, {make_bus(id10, -5.0, 0.0), make_bus(id20, 20.0, 0.0)});
  const auto r1 = tracker.update(objects1, ego);
  ASSERT_TRUE(r1.locked_uuid.has_value());
  EXPECT_EQ(r1.locked_uuid->uuid, id20.uuid);
}

}  // namespace autoware::following_goal_publisher::testing

