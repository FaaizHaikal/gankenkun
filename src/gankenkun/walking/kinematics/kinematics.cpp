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

#include "gankenkun/walking/kinematics/kinematics.hpp"

#include "jitsuyo/config.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

namespace gankenkun
{

Kinematics::Kinematics()
: ankle_length(0.0), calf_length(0.0), thigh_length(0.0), x_offset(0.0), y_offset(0.0)
{
  reset_angles();
}

void Kinematics::reset_angles()
{
  for (auto & angle : angles) {
    angle = 0_deg;
  }

  using tachimawari::joint::JointId;

  angles[JointId::NECK_YAW] = 0.0_deg;
  angles[JointId::NECK_PITCH] = 0.0_deg;
}

void Kinematics::set_config(const nlohmann::json & kinematic_data)
{
  bool valid_config = true;

  nlohmann::json leg_section;
  if (jitsuyo::assign_val(kinematic_data, "leg", leg_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(leg_section, "ankle_length", ankle_length);
    valid_section &= jitsuyo::assign_val(leg_section, "calf_length", calf_length);
    valid_section &= jitsuyo::assign_val(leg_section, "thigh_length", thigh_length);

    if (!valid_section) {
      std::cout << "Error found at section `leg`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json offset_section;
  if (jitsuyo::assign_val(kinematic_data, "offset", offset_section)) {
    bool valid_section = true;

    valid_section &= jitsuyo::assign_val(offset_section, "x", x_offset);
    valid_section &= jitsuyo::assign_val(offset_section, "y", y_offset);

    if (!valid_section) {
      std::cout << "Error found at section `offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }
}

void Kinematics::solve_inverse_kinematics(const Foot & left_foot, const Foot & right_foot)
{
  using tachimawari::joint::JointId;

  double left_x = left_foot.position.x - x_offset;
  double left_y = left_foot.position.y - y_offset;
  double left_z = ankle_length + calf_length + thigh_length - left_foot.position.z;

  double left_x2 = left_x * left_foot.yaw.cos() + left_y * left_foot.yaw.sin();
  double left_y2 = -left_x * left_foot.yaw.sin() + left_y * left_foot.yaw.cos();
  double left_z2 = left_z - ankle_length;

  // Hip roll angle
  keisan::Angle<double> hip_roll = keisan::signed_arctan(left_y2, left_z2);

  double left2 = left_y2 * left_y2 + left_z2 * left_z2;
  double left_z3 = std::sqrt(std::max(0.0, left2 - left_x2 * left_x2));

  keisan::Angle<double> pitch = keisan::signed_arctan(left_x2, left_z3);
  double length = std::hypot(left_x2, left_z3);
  keisan::Angle<double> knee_disp =
    keisan::arccos(keisan::clamp(length / (2.0 * thigh_length), -1.0, 1.0));

  // Hip pitch angle
  keisan::Angle<double> hip_pitch = -pitch - knee_disp;

  // Knee pitch angle
  keisan::Angle<double> knee_pitch = -pitch + knee_disp;

  angles[JointId::LEFT_HIP_YAW] = left_foot.yaw;
  angles[JointId::LEFT_HIP_ROLL] = hip_roll;
  angles[JointId::LEFT_HIP_PITCH] = -hip_pitch;
  angles[JointId::LEFT_KNEE] = -knee_pitch;
  angles[JointId::LEFT_ANKLE_PITCH] = 0.0_deg;   // TODO: Add offset from param left_foot pitch
  angles[JointId::LEFT_ANKLE_ROLL] = -hip_roll;  // TODO: Add offset from param left_foot roll

  std::cout << "Left Hip Yaw: " << angles[JointId::LEFT_HIP_YAW].degree() << std::endl;
  std::cout << "Left Hip Roll: " << angles[JointId::LEFT_HIP_ROLL].degree() << std::endl;
  std::cout << "Left Hip Pitch: " << angles[JointId::LEFT_HIP_PITCH].degree() << std::endl;
  std::cout << "Left Knee: " << angles[JointId::LEFT_KNEE].degree() << std::endl;
  std::cout << "Left Ankle Pitch: " << angles[JointId::LEFT_ANKLE_PITCH].degree() << std::endl;
  std::cout << "Left Ankle Roll: " << angles[JointId::LEFT_ANKLE_ROLL].degree() << std::endl;

  double right_x = right_foot.position.x - x_offset;
  double right_y = right_foot.position.y + y_offset;
  double right_z = ankle_length + calf_length + knee_length + thigh_length - right_foot.position.z;

  double right_x2 = right_x * right_foot.yaw.cos() + right_y * right_foot.yaw.sin();
  double right_y2 = -right_x * right_foot.yaw.sin() + right_y * right_foot.yaw.cos();
  double right_z2 = right_z - ankle_length;

  // Hip roll angle
  hip_roll = keisan::signed_arctan(right_y2, right_z2);

  double right2 = right_y2 * right_y2 + right_z2 * right_z2;
  double right_z3 = std::sqrt(std::max(0.0, right2 - right_x2 * right_x2));

  pitch = keisan::signed_arctan(right_x2, right_z3);
  length = std::hypot(right_x2, right_z3);
  knee_disp = keisan::arccos(keisan::clamp(length / (2.0 * thigh_length), -1.0, 1.0));

  // Hip pitch angle
  hip_pitch = -pitch - knee_disp;

  // Knee pitch angle
  knee_pitch = -pitch + knee_disp;

  angles[JointId::RIGHT_HIP_YAW] = right_foot.yaw;
  angles[JointId::RIGHT_HIP_ROLL] = hip_roll;
  angles[JointId::RIGHT_HIP_PITCH] = hip_pitch;
  angles[JointId::RIGHT_KNEE] = -knee_pitch;
  angles[JointId::RIGHT_ANKLE_PITCH] = 0.0_deg;   // TODO: Add offset from param right_foot pitch
  angles[JointId::RIGHT_ANKLE_ROLL] = -hip_roll;  // TODO: Add offset from param right_foot roll

  std::cout << "Right Hip Yaw: " << angles[JointId::RIGHT_HIP_YAW].degree() << std::endl;
  std::cout << "Right Hip Roll: " << angles[JointId::RIGHT_HIP_ROLL].degree() << std::endl;
  std::cout << "Right Hip Pitch: " << angles[JointId::RIGHT_HIP_PITCH].degree() << std::endl;
  std::cout << "Right Knee: " << angles[JointId::RIGHT_KNEE].degree() << std::endl;
  std::cout << "Right Ankle Pitch: " << angles[JointId::RIGHT_ANKLE_PITCH].degree() << std::endl;
  std::cout << "Right Ankle Roll: " << angles[JointId::RIGHT_ANKLE_ROLL].degree() << std::endl;
}

}  // namespace gankenkun
