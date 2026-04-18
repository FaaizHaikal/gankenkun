// Copyright (c) 2025 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef GANKENKUN__WALKING__NODE__WALKING_NODE_HPP_
#define GANKENKUN__WALKING__NODE__WALKING_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gankenkun/walking/node/walking_manager.hpp"
#include "gankenkun_interfaces/msg/point2.hpp"
#include "gankenkun_interfaces/msg/set_walking.hpp"
#include "gankenkun_interfaces/msg/status.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace gankenkun
{

class WalkingNode
{
public:
  using SetJoints = tachimawari_interfaces::msg::SetJoints;
  using WalkingStatus = gankenkun_interfaces::msg::Status;
  using Point2 = gankenkun_interfaces::msg::Point2;
  using KanseiStatus = kansei_interfaces::msg::Status;
  using SetWalking = gankenkun_interfaces::msg::SetWalking;

  WalkingNode(
    const rclcpp::Node::SharedPtr & node, const std::shared_ptr<WalkingManager> & walking_manager);

  void update();

private:
  void publish_joints();
  void publish_status();

  std::shared_ptr<WalkingManager> walking_manager;

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<SetWalking>::SharedPtr set_walking_subscriber;
  rclcpp::Subscription<Point2>::SharedPtr set_odometry_subscriber;
  rclcpp::Subscription<KanseiStatus>::SharedPtr orientation_subscriber;

  rclcpp::Publisher<SetJoints>::SharedPtr set_joints_publisher;
  rclcpp::Publisher<WalkingStatus>::SharedPtr status_publisher;

  rclcpp::CallbackGroup::SharedPtr set_walking_group;
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__NODE__WALKING_NODE_HPP_
