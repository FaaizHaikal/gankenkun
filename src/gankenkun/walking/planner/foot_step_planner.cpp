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

#include "gankenkun/walking/planner/foot_step_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "jitsuyo/jitsuyo.hpp"

using namespace keisan::literals;

namespace gankenkun
{

void FootStepPlanner::initialize(const std::string & path, double period)
{
  try {
    ov::Core core;
    auto m = core.read_model(path + "model.xml");
    model = core.compile_model(m, "CPU");
    infer_request = model.create_infer_request();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to load OpenVINO model: " + std::string(e.what()));
  }

  this->period = period;
  printf("Successfully initialized OpenVINO model\n");
}

void FootStepPlanner::set_config(const nlohmann::json & walking_data)
{
  bool valid_config = true;
  nlohmann::json planner_section;
  if (jitsuyo::assign_val(walking_data, "planner", planner_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(planner_section, "max_forward_stride", max_forward_stride);
    valid_section &=
      jitsuyo::assign_val(planner_section, "max_backward_stride", max_backward_stride);
    valid_section &= jitsuyo::assign_val(planner_section, "max_left_stride", max_left_stride);
    valid_section &= jitsuyo::assign_val(planner_section, "max_right_stride", max_right_stride);
    valid_section &= jitsuyo::assign_val(planner_section, "max_rotation", max_rotation);
    valid_section &= jitsuyo::assign_val(planner_section, "action_scale", action_scale);
    valid_section &= jitsuyo::assign_val(planner_section, "max_obstacles", max_obstacles);
    valid_section &= jitsuyo::assign_val(planner_section, "foot_length", foot_length);
    valid_section &= jitsuyo::assign_val(planner_section, "foot_width", foot_width);
    valid_section &= jitsuyo::assign_val(planner_section, "feet_spacing", feet_spacing);
    valid_section &= jitsuyo::assign_val(planner_section, "max_steps", max_steps);

    if (!valid_section) {
      std::cout << "Error found at section `planner`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `walking.json`");
  }
}

std::vector<float> FootStepPlanner::build_observation(
  const keisan::Point2 & support_pos, const keisan::Angle<double> & support_yaw,
  bool is_right_support, const keisan::Point2 & target_pos,
  const keisan::Angle<double> & target_yaw)
{
  std::vector<float> obs;

  double c = std::cos(support_yaw.radian());
  double s = std::sin(support_yaw.radian());

  // Transform target into support frame
  double dx = target_pos.x - support_pos.x;
  double dy = target_pos.y - support_pos.y;

  double x = c * dx + s * dy;
  double y = -s * dx + c * dy;

  double dtheta = (target_yaw - support_yaw).normalize().radian();

  double ct = std::cos(dtheta);
  double st = std::sin(dtheta);

  bool target_is_right = false;  // TODO: Infer from walking manager
  bool is_target_foot = (is_right_support == target_is_right);

  // Symmetry (LEFT FOOT)
  if (!is_right_support) {
    y = -y;
    st = -st;
  }

  obs.push_back(static_cast<float>(x));
  obs.push_back(static_cast<float>(y));
  obs.push_back(static_cast<float>(ct));
  obs.push_back(static_cast<float>(st));
  obs.push_back(is_target_foot ? 1.0f : 0.0f);

  size_t count = 0;
  for (const auto & obstacle : obstacles) {
    if (count >= max_obstacles) break;

    double dxo = obstacle.position.x - support_pos.x;
    double dyo = obstacle.position.y - support_pos.y;

    double ox = c * dxo + s * dyo;
    double oy = -s * dxo + c * dyo;

    if (!is_right_support) {
      oy = -oy;
    }

    obs.push_back(static_cast<float>(ox));
    obs.push_back(static_cast<float>(oy));
    obs.push_back(static_cast<float>(obstacle.radius));

    count++;
  }

  while (count < max_obstacles) {
    obs.insert(obs.end(), {0.0f, 0.0f, 0.0f});
    count++;
  }

  return obs;
}

void FootStepPlanner::apply_action(
  keisan::Point2 & support_pos, keisan::Angle<double> & support_yaw, bool & is_right_support,
  const std::vector<float> & action)
{
  double dx = action[0] * action_scale;
  double dy = action[1] * action_scale;
  double dtheta = action[2] * action_scale;

  // Clamp
  dx = std::clamp(dx, -max_backward_stride, max_forward_stride);
  dy = std::clamp(dy, -max_right_stride, max_left_stride);
  dtheta = std::clamp(dtheta, -max_rotation.radian(), max_rotation.radian());

  // Symmetry (LEFT FOOT)
  if (!is_right_support) {
    dy = -dy;
    dtheta = -dtheta;
  }

  // Transform to world
  double c = std::cos(support_yaw.radian());
  double s = std::sin(support_yaw.radian());

  double wx = c * dx - s * dy;
  double wy = s * dx + c * dy;

  support_pos.x += wx;
  support_pos.y += (wy);

  support_yaw += keisan::make_radian(dtheta);

  // Switch support
  is_right_support = !is_right_support;
}

std::vector<float> FootStepPlanner::infer(const std::vector<float> & obs)
{
  ov::Tensor input_tensor(ov::element::f32, {1, obs.size()}, const_cast<float *>(obs.data()));
  infer_request.set_input_tensor(input_tensor);

  infer_request.infer();

  auto output = infer_request.get_output_tensor();
  const float * out = output.data<const float>();

  size_t output_size = output.get_size();

  // Debug Print
  // for (size_t i = 0; i < output_size; ++i) {
  //   std::cout << "out[" << i << "]: " << out[i] << "\n";
  // }

  return {out[0], out[1], out[2]};
}

void FootStepPlanner::plan_next_step(
  const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
  keisan::Point2 & support_pos, keisan::Angle<double> & support_yaw, bool & is_right_support,
  double & time)
{
  auto obs = build_observation(
    support_pos, support_yaw, is_right_support, target_position, target_orientation);

  auto action = infer(obs);

  apply_action(support_pos, support_yaw, is_right_support, action);

  time += period;
  foot_steps.push_back({time, support_pos, support_yaw, is_right_support ? RIGHT_FOOT : LEFT_FOOT});
}

void FootStepPlanner::plan_stop(
  keisan::Point2 current_position, keisan::Angle<double> & current_orientation)
{
  if (foot_steps.size() > 1) {
    foot_steps.erase(foot_steps.begin() + 1, foot_steps.end());
  }

  // force BOTH_FEET state
  foot_steps.push_back(
    {foot_steps.front().time + period, current_position, current_orientation,
     FootStepPlanner::BOTH_FEET});
}

void FootStepPlanner::print_foot_steps()
{
  size_t counter = 1;
  for (const auto & step : foot_steps) {
    std::string support = step.support_foot == RIGHT_FOOT  ? "right"
                          : step.support_foot == LEFT_FOOT ? "left"
                                                           : "both";

    std::cout << "Step " << counter++ << "-> Time(" << step.time << "); Position("
              << step.position.x << ", " << step.position.y << "); Rotation("
              << step.rotation.radian() << "); "
              << "Support(\'" << support << "\')\n";
  }
}

}  // namespace gankenkun
