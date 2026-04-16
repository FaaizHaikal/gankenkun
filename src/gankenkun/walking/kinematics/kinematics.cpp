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

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `kinematic.json`");
  }
}

void Kinematics::solve_inverse_kinematics(const Foot & left_foot, const Foot & right_foot)
{
  using tachimawari::joint::JointId;

  // ===================== LEFT LEG =====================
  double left_x = left_foot.position.x - x_offset;
  double left_y = left_foot.position.y - y_offset;
  double left_z = ankle_length + calf_length + thigh_length - left_foot.position.z;

  double left_x2 = left_x * left_foot.yaw.cos() + left_y * left_foot.yaw.sin();
  double left_y2 = -left_x * left_foot.yaw.sin() + left_y * left_foot.yaw.cos();
  double left_z2 = left_z - ankle_length;

  // Hip roll
  keisan::Angle<double> hip_roll = keisan::signed_arctan(left_y2, left_z2);

  double px = left_x2;
  double pz = left_z2;

  double length = std::hypot(px, pz);

  double L1 = thigh_length;
  double L2 = calf_length;

  // Clamp
  double length_clamped = keisan::clamp(length, 1e-6, L1 + L2 - 1e-6);
  double cos_knee = (L1 * L1 + L2 * L2 - length_clamped * length_clamped) / (2.0 * L1 * L2);

  keisan::Angle<double> knee_pitch =
    keisan::make_radian(M_PI - std::acos(keisan::clamp(cos_knee, -1.0, 1.0)));

  double cos_hip =
    (L1 * L1 + length_clamped * length_clamped - L2 * L2) / (2.0 * L1 * length_clamped);

  keisan::Angle<double> hip_pitch =
    keisan::signed_arctan(px, pz) -
    keisan::make_radian(std::acos(keisan::clamp(cos_hip, -1.0, 1.0)));

  keisan::Angle<double> ankle_pitch = hip_pitch + knee_pitch;

  angles[JointId::LEFT_HIP_YAW] = left_foot.yaw;
  angles[JointId::LEFT_HIP_ROLL] = hip_roll;
  angles[JointId::LEFT_HIP_PITCH] = -hip_pitch;
  angles[JointId::LEFT_KNEE] = -knee_pitch;
  angles[JointId::LEFT_ANKLE_PITCH] = -ankle_pitch;
  angles[JointId::LEFT_ANKLE_ROLL] = -hip_roll;

  // ===================== RIGHT LEG =====================
  double right_x = right_foot.position.x - x_offset;
  double right_y = right_foot.position.y + y_offset;
  double right_z = ankle_length + calf_length + thigh_length - right_foot.position.z;

  double right_x2 = right_x * right_foot.yaw.cos() + right_y * right_foot.yaw.sin();
  double right_y2 = -right_x * right_foot.yaw.sin() + right_y * right_foot.yaw.cos();
  double right_z2 = right_z - ankle_length;

  hip_roll = keisan::signed_arctan(right_y2, right_z2);

  px = right_x2;
  pz = right_z2;

  length = std::hypot(px, pz);
  length_clamped = keisan::clamp(length, 1e-6, L1 + L2 - 1e-6);

  cos_knee = (L1 * L1 + L2 * L2 - length_clamped * length_clamped) / (2.0 * L1 * L2);

  knee_pitch = keisan::make_radian(M_PI - std::acos(keisan::clamp(cos_knee, -1.0, 1.0)));

  cos_hip = (L1 * L1 + length_clamped * length_clamped - L2 * L2) / (2.0 * L1 * length_clamped);

  hip_pitch = keisan::signed_arctan(px, pz) -
              keisan::make_radian(std::acos(keisan::clamp(cos_hip, -1.0, 1.0)));

  ankle_pitch = hip_pitch + knee_pitch;

  angles[JointId::RIGHT_HIP_YAW] = right_foot.yaw;
  angles[JointId::RIGHT_HIP_ROLL] = hip_roll;
  angles[JointId::RIGHT_HIP_PITCH] = hip_pitch;
  angles[JointId::RIGHT_KNEE] = knee_pitch;
  angles[JointId::RIGHT_ANKLE_PITCH] = ankle_pitch;
  angles[JointId::RIGHT_ANKLE_ROLL] = -hip_roll;
}

}  // namespace gankenkun
