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

#ifndef GANKENKUN__WALKING__KINEMATICS__KINEMATICS_HPP_
#define GANKENKUN__WALKING__KINEMATICS__KINEMATICS_HPP_

#include <array>
#include <string>

#include "gankenkun/walking/planner/foot_step_planner.hpp"
#include "keisan/angle.hpp"
#include "keisan/geometry/point_3.hpp"
#include "nlohmann/json.hpp"

using namespace keisan::literals;

namespace gankenkun
{

class Kinematics
{
public:
  using FootStep = FootStepPlanner::FootStep;

  struct Foot
  {
    keisan::Point3 position;
    keisan::Angle<double> yaw;

    Foot() : position(keisan::Point3(0.0, 0.0, 0.0)), yaw(0.0_deg) {}
  };

  Kinematics();

  void reset_angles();
  void set_config(const nlohmann::json & kinematic_data);
  Foot forward_kinematics(int foot) const;
  void solve_inverse_kinematics(const Foot & left_foot, const Foot & right_foot);

  const std::array<keisan::Angle<double>, 23> & get_angles() const { return angles; }

private:
  double ankle_length;
  double calf_length;
  double knee_length;
  double thigh_length;

  double x_offset;
  double y_offset;

  std::array<keisan::Angle<double>, 23> angles;
};

}  // namespace gankenkun

#endif  // GANKENKUN__WALKING__KINEMATICS__KINEMATICS_HPP_
