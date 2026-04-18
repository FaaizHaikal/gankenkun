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

using namespace keisan::literals;

namespace gankenkun
{

FootStepPlanner::FootStepPlanner()
: period(0.0), width(0.0), max_stride(0.0, 0.0), max_rotation(0.0_deg)
{
}

void FootStepPlanner::set_parameters(
  const keisan::Point2 & max_stride, const keisan::Angle<double> & max_rotation, double period,
  double width)
{
  this->period = period;
  this->width = width;
  this->max_stride = max_stride;
  this->max_rotation = max_rotation;
}

void FootStepPlanner::plan(
  const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
  keisan::Point2 & current_position, keisan::Angle<double> & current_orientation, int next_support,
  int status)
{
  // Calculate the number of foot step
  double time = 0.0;

  double steps_x = std::abs((target_position.x - current_position.x) / max_stride.x);
  double steps_y = std::abs((target_position.y - current_position.y) / max_stride.y);
  double steps_angle =
    std::abs(((target_orientation - current_orientation).radian()) / max_rotation.radian());
  int max_steps = std::max(std::max(steps_x, steps_y), steps_angle);

  double stride_x = 0.0;
  double stride_y = 0.0;
  double stride_angle = 0.0;

  if (max_steps > 0) {
    stride_x = (target_position.x - current_position.x) / max_steps;
    stride_y = (target_position.y - current_position.y) / max_steps;
    stride_angle = (target_orientation - current_orientation).radian() / max_steps;
  }

  // Plan first foot step
  foot_steps.clear();
  if (status == START) {
    foot_steps.push_back({0.0, current_position, current_orientation, BOTH_FEET});
    time += period * 2;
  }

  if (next_support == LEFT_FOOT) {
    foot_steps.push_back(
      {time, keisan::Point2(current_position.x, current_position.y + width), current_orientation,
       LEFT_FOOT});
    next_support = RIGHT_FOOT;
  } else {
    foot_steps.push_back(
      {time, keisan::Point2(current_position.x, current_position.y - width), current_orientation,
       RIGHT_FOOT});
    next_support = LEFT_FOOT;
  }

  // Plan walking foot steps
  size_t counter = 0;
  while (true) {
    counter += 1;

    double delta_x = std::abs(target_position.x - current_position.x);
    double delta_y = std::abs(target_position.y - current_position.y);
    double delta_angle = std::abs((target_orientation - current_orientation).radian());

    if (delta_x < max_stride.x && delta_y < max_stride.y && delta_angle < max_rotation.radian()) {
      break;
    }

    time += period;

    auto next_position = current_position + keisan::Point2(stride_x, stride_y);
    auto next_orientation = current_orientation + keisan::make_radian(stride_angle);

    if (next_support == LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(next_position.x, next_position.y + width), next_orientation,
         LEFT_FOOT});
      next_support = RIGHT_FOOT;
    } else {
      foot_steps.push_back(
        {time, keisan::Point2(next_position.x, next_position.y - width), next_orientation,
         RIGHT_FOOT});
      next_support = LEFT_FOOT;
    }

    current_position = next_position;
    current_orientation = next_orientation;
  }

  // Planning walk in position
  if (status != STOP) {
    time += period;

    if (next_support == LEFT_FOOT) {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y + width), target_orientation,
         LEFT_FOOT});
    } else {
      foot_steps.push_back(
        {time, keisan::Point2(target_position.x, target_position.y - width), target_orientation,
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
