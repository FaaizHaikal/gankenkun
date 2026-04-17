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

#include "gankenkun/lipm/lipm.hpp"

namespace gankenkun
{

LIPM::LIPM() : dt(0.0), period(0.0), z(0.0) {}

void LIPM::set_parameters(double z, double dt, double period, double kp, double kd)
{
  this->z = z;
  this->dt = dt;
  this->period = period;
  this->balance_kp = kp;
  this->balance_kd = kd;

  initialize();
  solve_dare();
}

// Initialize the discrete-time LTI system matrices
void LIPM::initialize()
{
  // Initialize outputs
  x_state = keisan::Matrix<3, 1>::zero();
  y_state = keisan::Matrix<3, 1>::zero();

  velocity.x = 0.0;
  velocity.y = 0.0;

  // Continuous-time system matrices
  auto A = keisan::Matrix<3, 3>(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);

  auto B = keisan::Matrix<3, 1>(0.0, 0.0, 1.0);

  auto C = keisan::Matrix<1, 3>(1.0, 0.0, -z / 9.8);

  // Discretize the continuous-time system
  A_d = A.exp(dt);

  B_d = keisan::Matrix<3, 1>::zero();

  for (size_t i = 0; i < 10; ++i) {
    double tau = dt * (i + 0.5) / 10;
    auto expA = A.exp(tau);
    auto expAB = expA * B;
    B_d += expAB * dt / 10;
  }

  C_d = C;
}

// Solve the discrete-time algebraic Riccati equation
void LIPM::solve_dare()
{
  auto E_d = keisan::Matrix<3, 1>(dt, 1.0, 0.0);

  auto CA_d = -C_d * A_d;
  auto Phai = keisan::Matrix<4, 4>(
    1.0, CA_d[0][0], CA_d[0][1], CA_d[0][2], 0.0, A_d[0][0], A_d[0][1], A_d[0][2], 0.0, A_d[1][0],
    A_d[1][1], A_d[1][2], 0.0, A_d[2][0], A_d[2][1], A_d[2][2]);

  auto CB_d = -C_d * B_d;
  auto G = keisan::Matrix<4, 1>(CB_d[0][0], B_d[0][0], B_d[1][0], B_d[2][0]);

  auto GR = keisan::Matrix<4, 1>(1.0, 0.0, 0.0, 0.0);

  auto Qm = keisan::Matrix<4, 4>::zero();
  Qm[0][0] = 1.0e+8;

  auto H = keisan::Matrix<1, 1>(1.0);

  // Iterative DARE solver
  auto P = Qm;

  const double tolerance = 1e-9;
  const size_t max_iterations = 1000;

  for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
    auto P_prev = P;

    auto GTP = G.transpose() * P;

    auto T = (H + GTP * G);

    if (T.inverse()) {
      auto K = T * GTP * Phai;

      P = Phai.transpose() * P * Phai - Phai.transpose() * P * G * K + Qm;

      // Check for convergence
      if ((P - P_prev).norm() < tolerance) {
        break;
      }
    } else {
      throw std::runtime_error("Failed to solve DARE equation!");
    }
  }

  // Extract the feedback gain
  auto GTP = G.transpose() * P;
  auto T = (H + GTP * G);

  if (T.inverse()) {
    F = (-T) * GTP * Phai;
  } else {
    throw std::runtime_error("Failed to extract feedback gain!");
  }

  // Extract the preview gain
  auto I = keisan::Matrix<4, 4>::identity();
  auto xiT = ((I - G * T * GTP) * Phai).transpose();

  for (int i = 0; i < static_cast<int>(round(period / dt)); i++) {
    auto fi = (-T) * G.transpose() * xiT.power(i - 1) * P * GR;

    f.push_back(fi[0][0]);
  }
}

// Update the LIPM state
void LIPM::update(double time, const std::deque<FootStepPlanner::FootStep> & foot_steps, bool reset)
{
  if (reset) {
    velocity.x = 0.0;
    velocity.y = 0.0;
  }

  // Safety: need at least current + next step
  if (foot_steps.size() < 2) {
    return;
  }

  com_trajectory.clear();

  auto next_x_state = x_state;
  auto next_y_state = y_state;

  // Duration of ONE step only
  int horizon = static_cast<int>(round((foot_steps[1].time - time) / dt));

  // Target = NEXT foot (NOT front!)
  double target_x = foot_steps[1].position.x;
  double target_y = foot_steps[1].position.y;

  for (int i = 0; i < horizon; i++) {
    auto projected_x = C_d * next_x_state;
    auto projected_y = C_d * next_y_state;

    double zmp_x = projected_x[0][0];
    double zmp_y = projected_y[0][0];

    double error_x = target_x - zmp_x;
    double error_y = target_y - zmp_y;

    double acc_x = balance_kp * error_x - balance_kd * velocity.x;
    double acc_y = balance_kp * error_y - balance_kd * velocity.y;

    // Integrate velocity
    velocity.x += acc_x * dt;
    velocity.y += acc_y * dt;

    // Update states
    x_state = next_x_state;
    y_state = next_y_state;

    next_x_state = A_d * x_state + B_d * velocity.x;
    next_y_state = A_d * y_state + B_d * velocity.y;

    COMTrajectory com;

    com.position.x = next_x_state[0][0];
    com.position.y = next_y_state[0][0];
    com.projected_position.x = zmp_x;
    com.projected_position.y = zmp_y;

    com_trajectory.push_back(com);
  }
}

// Pop the front of the COM trajectory
LIPM::COMTrajectory LIPM::pop_front()
{
  auto com = com_trajectory.front();
  com_trajectory.pop_front();

  return com;
}

}  // namespace gankenkun
