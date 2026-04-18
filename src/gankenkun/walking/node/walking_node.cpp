// Copyright (c) 2025 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "gankenkun/walking/node/walking_node.hpp"

#include "tachimawari/joint/utils/middleware.hpp"

namespace gankenkun
{

WalkingNode::WalkingNode(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<WalkingManager> & walking_manager)
: node(node), walking_manager(walking_manager)
{
  set_walking_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = set_walking_group;

  set_walking_subscriber = node->create_subscription<SetWalking>(
    "walking/set_walking", 10,
    [this](const SetWalking::SharedPtr message) {
      while (true) {
        if (this->walking_manager->replan()) {
          if (message->run) {
            this->walking_manager->set_goal(
              keisan::Point2(message->position.x / -100.0, message->position.y / -100.0),
              keisan::make_degree(message->orientation));
          } else {
            this->walking_manager->stop();
          }

          return;
        }
      }
    },
    subscriber_option);

  set_odometry_subscriber = node->create_subscription<Point2>(
    "walking/set_odometry", 10, [this](const Point2::SharedPtr message) {
      this->walking_manager->set_position(keisan::Point2(message->x / -100.0, message->y / -100.0));
    });

  orientation_subscriber = node->create_subscription<KanseiStatus>(
    "measurement/status", 10, [this](const KanseiStatus::SharedPtr message) {
      this->walking_manager->set_orientation(keisan::make_degree(message->orientation.yaw));
    });

  status_publisher = node->create_publisher<WalkingStatus>("walking/status", 10);

  set_joints_publisher = node->create_publisher<SetJoints>("joint/set_joints", 10);
}

void WalkingNode::update()
{
  publish_joints();
  publish_status();
}

void WalkingNode::publish_joints()
{
  auto joints_msg = SetJoints();

  const auto & joints = walking_manager->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  joints_msg.control_type = tachimawari::joint::Middleware::FOR_WALKING;

  set_joints_publisher->publish(joints_msg);
}

void WalkingNode::publish_status()
{
  auto status_msg = WalkingStatus();

  status_msg.is_running = walking_manager->is_running();
  status_msg.odometry.x = walking_manager->get_position().x * -100.0;
  status_msg.odometry.y = walking_manager->get_position().y * -100.0;

  status_publisher->publish(status_msg);
}

}  // namespace gankenkun
