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

#ifndef GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_
#define GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_

#include <nlohmann/json.hpp>

#include "gankenkun/lipm/lipm.hpp"
#include "gankenkun/walking/kinematics/kinematics.hpp"
#include "gankenkun/walking/planner/foot_step_planner.hpp"
#include "tachimawari/joint/joint.hpp"

namespace gankenkun
{

class WalkingManager
{
public:
  using FootStep = FootStepPlanner::FootStep;

  WalkingManager();

  void load_config(const std::string & path);
  void set_config(
    const nlohmann::json & walking_data, const nlohmann::json & kinematic_data,
    const nlohmann::json & planner_data);

  void stop();
  void update_time();
  void update_joints();
  void process();
  bool replan();

  const std::vector<tachimawari::joint::Joint> & get_joints() const { return joints; }

  void remove_steps();
  void set_goal(
    const keisan::Point2 & goal_position, const keisan::Angle<double> & goal_orientation);

  void set_position(const keisan::Point2 & position);
  void set_orientation(const keisan::Angle<double> & orientation);

  const keisan::Point2 & get_position() const { return robot_position; }
  bool is_running();
  void update_odometry(int current_support);

private:
  keisan::Angle<double> get_balance_body_pitch() const;

  Kinematics kinematics;
  LIPM lipm;
  FootStepPlanner foot_step_planner;

  int status;
  bool initialized;
  int next_support;
  int previous_support;
  keisan::Point2 robot_position;
  keisan::Angle<double> robot_orientation;
  keisan::Point2 support_foot_position;

  // Timing parameters
  double time_step;
  double dsp_duration;
  double plan_period;
  double step_frames;
  double com_period;

  // Posture parameters
  double com_height;
  double foot_height;
  double feet_lateral;
  keisan::Angle<double> forward_lean;
  double forward_lean_ratio;
  keisan::Angle<double> backward_lean;
  double backward_lean_ratio;
  keisan::Angle<double> left_shoulder_roll;
  keisan::Angle<double> left_shoulder_pitch;
  keisan::Angle<double> left_elbow;
  keisan::Angle<double> right_shoulder_roll;
  keisan::Angle<double> right_shoulder_pitch;
  keisan::Angle<double> right_elbow;

  // Offset parameters
  keisan::Point3 foot_offset;
  double step_y_offset;
  keisan::Point2 odometry_offset;

  // Maximum stride parameters
  keisan::Point2 max_stride;
  keisan::Angle<double> max_rotation;

  double left_up;
  double right_up;

  std::vector<tachimawari::joint::Joint> joints;

  keisan::Matrix<1, 3> left_offset = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> left_offset_delta = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> left_foot_target = keisan::Matrix<1, 3>::zero();

  keisan::Matrix<1, 3> right_offset = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> right_offset_delta = keisan::Matrix<1, 3>::zero();
  keisan::Matrix<1, 3> right_foot_target = keisan::Matrix<1, 3>::zero();
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__NODE__WALKING_MANAGER_HPP_
