// Copyright (c) 2025 ICHIRO ITS
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

#include "gankenkun/config/node/config_node.hpp"

#include "jitsuyo/config.hpp"

namespace gankenkun
{

ConfigNode::ConfigNode(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<WalkingManager> & walking_manager,
  const std::string & path)
: node(node), walking_manager(walking_manager)
{
  get_config_server = node->create_service<GetConfig>(
    "gankenkun/config/get_config",
    [this, path](GetConfig::Request::SharedPtr request, GetConfig::Response::SharedPtr response) {
      nlohmann::ordered_json walking_data;
      nlohmann::ordered_json kinematic_data;
      if (!jitsuyo::load_config(path, "walking.json", walking_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Get config server"), "Failed to load walking config");
        return;
      }

      if (!jitsuyo::load_config(path, "kinematic.json", kinematic_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Get config server"), "Failed to load kinematic config");
        return;
      }

      response->json_walking = walking_data.dump();
      response->json_kinematic = kinematic_data.dump();
    });

  update_config_server = node->create_service<UpdateConfig>(
    "gankenkun/config/update_config",
    [this, path](
      UpdateConfig::Request::SharedPtr request, UpdateConfig::Response::SharedPtr response) {
      nlohmann::ordered_json walking_data;
      nlohmann::ordered_json kinematic_data;
      nlohmann::ordered_json planner_data;

      walking_data = nlohmann::ordered_json::parse(request->json_walking);
      kinematic_data = nlohmann::ordered_json::parse(request->json_kinematic);
      planner_data = nlohmann::ordered_json::parse(request->json_planner);

      if (request->save) {
        if (!jitsuyo::save_config(path, "walking.json", walking_data)) {
          RCLCPP_ERROR(rclcpp::get_logger("Update config server"), "Failed to save walking config");

          response->ok = false;
          return;
        }

        if (!jitsuyo::save_config(path, "kinematic.json", kinematic_data)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("Update config server"), "Failed to save kinematic config");

          response->ok = false;
          return;
        }

        if (!jitsuyo::save_config(path, "planner.json", planner_data)) {
          RCLCPP_ERROR(rclcpp::get_logger("Update config server"), "Failed to save planner config");

          response->ok = false;
          return;
        }

        RCLCPP_INFO(rclcpp::get_logger("Update config server"), "Config saved");
      } else {
        this->walking_manager->set_config(walking_data, kinematic_data, planner_data);

        RCLCPP_INFO(rclcpp::get_logger("Update config server"), "Config updated");
      }

      response->ok = true;
    });
}

}  // namespace gankenkun
