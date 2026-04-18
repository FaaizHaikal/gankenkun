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

#include "jitsuyo/jitsuyo.hpp"

using namespace keisan::literals;

namespace gankenkun
{

void FootStepPlanner::initialize(const std::string & path)
{
  try {
    auto model = core.read_model(path + "model.xml");
    compiled_model = core.compile_model(model, "CPU");
    infer_request = compiled_model.create_infer_request();
  } catch (const std::exception & e) {
    throw("Failed to load OpenVINO model: %s", e.what());
  }
}

void FootStepPlanner::set_config(const nlohmann::json & planner_data)
{
  bool valid_config = true;

  nlohmann::json stride_section;
  if (jitsuyo::assign_val(planner_data, "stride", stride_section)) {
    bool valid_section = jitsuyo::assign_val(stride_section, "max_forward", max_forward);
    valid_section &= jitsuyo::assign_val(stride_section, "max_backward", max_backward);
    valid_section &= jitsuyo::assign_val(stride_section, "max_left", max_left);
    valid_section &= jitsuyo::assign_val(stride_section, "max_right", max_right);
    valid_section &= jitsuyo::assign_val(stride_section, "max_rotation", max_rotation);
    valid_section &= jitsuyo::assign_val(stride_section, "action_scale", action_scale);

    if (!valid_section) {
      std::cout << "Error found at section `stride`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json foot_geometry_section;
  if (jitsuyo::assign_val(planner_data, "foot_geometry", foot_geometry_section)) {
    bool valid_section = jitsuyo::assign_val(foot_geometry_section, "foot_width", foot_width);
    valid_section &= jitsuyo::assign_val(foot_geometry_section, "foot_length", foot_length);
    valid_section &= jitsuyo::assign_val(foot_geometry_section, "feet_spacing", feet_spacing);

    if (!valid_section) {
      std::cout << "Error found at section `foot_geometry`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json target_section;
  if (jitsuyo::assign_val(planner_data, "target", target_section)) {
    bool valid_section =
      jitsuyo::assign_val(target_section, "distance_tolerance", distance_tolerance);
    valid_section &=
      jitsuyo::assign_val(target_section, "direction_tolerance", direction_tolerance);

    if (!valid_section) {
      std::cout << "Error found at section `target`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json obstacle_section;
  if (jitsuyo::assign_val(planner_data, "obstacle", obstacle_section)) {
    bool valid_section = jitsuyo::assign_val(obstacle_section, "max_count", max_obstacle);
    valid_section &= jitsuyo::assign_val(obstacle_section, "max_radius", max_obstacle_radius);

    if (!valid_section) {
      std::cout << "Error found at section `obstacle`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `planner.json`");
  }
}

bool FootStepPlanner::is_reached_target(
  const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
  keisan::Point2 & current_position, keisan::Angle<double> & current_orientation)
{
  double dist = (target_position - current_position).magnitude();
  double dir = (target_orientation - current_orientation).normalize().degree();

  return dist <= distance_tolerance && std::abs(dir) <= direction_tolerance;
}

std::vector<float> FootStepPlanner::infer(const std::vector<float> & obs)
{
  ov::Tensor input_tensor(ov::element::f32, {1, obs.size()}, const_cast<float *>(obs.data()));
  infer_request.set_input_tensor(input_tensor);

  infer_request.infer();

  auto output = infer_request.get_output_tensor();
  const float * out = output.data<const float>();

  return {out[0], out[1], out[2]};
}

std::vector<float> FootStepPlanner::build_observation(
  const keisan::Point2 & target_pos, const keisan::Angle<double> & target_yaw,
  const keisan::Point2 & support_pos, const keisan::Angle<double> & support_yaw, int next_support)
{
  std::vector<float> obs;

  double c = support_yaw.cos();
  double s = support_yaw.sin();

  // Transform target into support frame
  double dx = target_pos.x - support_pos.x;
  double dy = target_pos.y - support_pos.y;

  double x = c * dx + s * dy;
  double y = -s * dx + c * dy;

  auto dtheta = (target_yaw - support_yaw).normalize(0.0, 360.0);

  double ct = dtheta.cos();
  double st = dtheta.sin();
  bool is_right_support = next_support == RIGHT_FOOT;
  bool same_stop_foot = false;

  // Symmetry (LEFT FOOT)
  if (!is_right_support) {
    y = -y;
    st = -st;
  }

  obs.push_back(static_cast<float>(x));
  obs.push_back(static_cast<float>(y));
  obs.push_back(static_cast<float>(ct));
  obs.push_back(static_cast<float>(st));
  obs.push_back(same_stop_foot ? 1.0f : 0.0f);

  size_t count = 0;
  for (const auto & obstacle : obstacles) {
    if (count >= max_obstacle) break;

    double dxo = obstacle.position.x - support_pos.x;
    double dyo = obstacle.position.y - support_pos.y;

    double ox = c * dxo + s * dyo;
    double oy = -s * dxo + c * dyo;

    if (!is_right_support) {
      oy = -oy;
    }

    obs.push_back(static_cast<float>(ox));
    obs.push_back(static_cast<float>(oy));
    obs.push_back(static_cast<float>(std::min(obstacle.radius, max_obstacle_radius)));

    count++;
  }

  while (count < max_obstacle) {
    obs.insert(obs.end(), {0.0f, 0.0f, 0.0f});
    count++;
  }

  return obs;
}

void FootStepPlanner::apply_action(
  keisan::Point2 & support_pos, keisan::Angle<double> & support_yaw, int next_support,
  const std::vector<float> & action)
{
  double dx = action[0] * action_scale;
  double dy = action[1] * action_scale + feet_spacing;
  double dtheta = action[2] * action_scale;

  // Clamp
  dx = keisan::clamp(dx, -max_backward, max_forward);
  dy = keisan::clamp(dy, -max_right, max_left);
  dtheta = keisan::clamp(dtheta, -max_rotation.radian(), max_rotation.radian());
  // printf("dx: %f, dy: %f, dtheta: %f\n", dx, dy, dtheta);
  bool is_right_support = next_support == RIGHT_FOOT;

  // Symmetry (LEFT FOOT)
  if (!is_right_support) {
    dy = -dy;
    dtheta = -dtheta;
  }

  // Transform to world
  double c = support_yaw.cos();
  double s = support_yaw.sin();

  double wx = c * dx - s * dy;
  double wy = s * dx + c * dy;

  support_pos.x += wx;
  support_pos.y += wy;

  support_yaw += keisan::make_radian(dtheta).normalize();
  // printf("sup yaw: %f\n", support_yaw.degree());
}

void FootStepPlanner::plan(
  const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
  keisan::Point2 & current_position, keisan::Angle<double> & current_orientation, int next_support,
  int status)
{
  // Calculate the number of foot step
  double time = 0.0;

  // Plan first foot step
  foot_steps.clear();
  if (status == START) {
    foot_steps.push_back({0.0, current_position, current_orientation, BOTH_FEET});
    time += period * 2;
  }

  if (
    is_reached_target(target_position, target_orientation, current_position, current_orientation)) {
    std::cout << "reached target\n";
    foot_steps.push_back({time, current_position, current_orientation, BOTH_FEET});
    time += period;
    foot_steps.push_back({time, current_position, current_orientation, BOTH_FEET});
  } else {
    if (next_support == LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(current_position.x, current_position.y), current_orientation,
         LEFT_FOOT});
      next_support = RIGHT_FOOT;
    } else {
      foot_steps.push_back(
        {time, keisan::Point2(current_position.x, current_position.y), current_orientation,
         RIGHT_FOOT});
      next_support = LEFT_FOOT;
    }
  }

  // Plan walking foot steps
  size_t counter = 0;
  const size_t max_episode_steps = 1000;
  while (counter < max_episode_steps) {
    counter += 1;

    if (
      is_reached_target(
        target_position, target_orientation, current_position, current_orientation)) {
      break;
    }

    auto obs = build_observation(
      target_position, target_orientation, current_position, current_orientation, next_support);

    auto action = infer(obs);

    apply_action(current_position, current_orientation, next_support, action);

    time += period;
    foot_steps.push_back({time, current_position, current_orientation, next_support});
    next_support = (next_support == RIGHT_FOOT ? LEFT_FOOT : RIGHT_FOOT);
  }

  // Planning walk in position
  if (status != STOP) {
    time += period;

    if (next_support == LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y), target_orientation,
         LEFT_FOOT});
    } else if (next_support == LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y), target_orientation,
         RIGHT_FOOT});
    }

    time += period;
    next_support = BOTH_FEET;
    foot_steps.push_back({time, target_position, target_orientation, BOTH_FEET});

    time += 2.0 * period;
    foot_steps.push_back({time, target_position, target_orientation, BOTH_FEET});

    time += 100.0;
    foot_steps.push_back({time, target_position, target_orientation, BOTH_FEET});
  }
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
              << step.rotation.degree() << "); "
              << "Support(\'" << support << "\')\n";
  }
}

}  // namespace gankenkun
