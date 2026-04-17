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

#ifndef GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_
#define GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_

#include <deque>
#include <nlohmann/json.hpp>
#include <openvino/openvino.hpp>
#include <optional>
#include <string>

#include "keisan/angle.hpp"
#include "keisan/geometry/point_2.hpp"
#include "keisan/matrix/matrix.hpp"

namespace gankenkun
{

class FootStepPlanner
{
public:
  enum { LEFT_FOOT = 0, RIGHT_FOOT = 1, BOTH_FEET = 2 };

  enum { STOP = 0, WALKING = 1 };

  struct Obstacle
  {
    keisan::Point2 position;
    double radius;
  };

  struct FootStep
  {
    double time;
    keisan::Point2 position;
    keisan::Angle<double> rotation;
    int support_foot;
  };

  FootStepPlanner() = default;

  void plan_next_step(
    const keisan::Point2 & target_position, const keisan::Angle<double> & target_orientation,
    keisan::Point2 & support_pos, keisan::Angle<double> & support_yaw, bool & is_right_support,
    double & time);
  void plan_stop(keisan::Point2 current_position, keisan::Angle<double> & current_orientation);

  void initialize(const std::string & path, double period);
  void set_config(const nlohmann::json & walking_data);

  std::vector<float> build_observation(
    const keisan::Point2 & support_pos, const keisan::Angle<double> & support_yaw,
    bool is_right_support, const keisan::Point2 & target_pos,
    const keisan::Angle<double> & target_yaw);
  std::vector<float> infer(const std::vector<float> & obs);

  void apply_action(
    keisan::Point2 & support_pos, keisan::Angle<double> & support_yaw, bool & is_right_support,
    const std::vector<float> & action);
  void print_foot_steps();

  double get_max_forward_stride() const { return this->max_forward_stride; }
  double get_max_backward_stride() const { return this->max_backward_stride; }

  std::deque<FootStep> foot_steps;

private:
  // Steps
  double max_forward_stride;
  double max_backward_stride;
  double max_left_stride;
  double max_right_stride;
  keisan::Angle<double> max_rotation;

  // Target tolerance
  double distance_tolerance;
  double orientation_tolerance;
  double action_scale;

  // Obstacles
  size_t max_obstacles;
  std::vector<Obstacle> obstacles;

  // Foot geometry
  double foot_length;
  double foot_width;
  double feet_spacing;

  // Timing
  double period;
  int max_steps;

  // Environments
  std::vector<float> observations;

  // OpenVINO components
  ov::CompiledModel model;
  ov::InferRequest infer_request;
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__PLANNER__FOOT_STEP_PLANNER_HPP_
