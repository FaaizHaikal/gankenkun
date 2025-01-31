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

#ifndef GANKENKUN__LIPM__LIPM_HPP_
#define GANKENKUN__LIPM__LIPM_HPP_

#include "keisan/matrix.hpp"

namespace gankenkun
{

class LIPM
{
public:
  LIPM();
  ~LIPM() {}

  void initialize();
  void solve_dare();

  void update(
    double time, const std::list<FootStepPlanner::FootStep> & foot_steps, bool reset = false);

  COMTrajectory pop_front();

  const std::vector<COMTrajectory> & get_com_trajectory() const { return com_trajectory; }

  double dt;
  double period;
  double z;

  struct COMTrajectory
  {
    keisan::Point2 position;
    keisan::Point2 projected_position;
  };

private:
  // Discrete-time system matrices
  keisan::Matrix<3, 3> A_d;
  keisan::Matrix<3, 1> B_d;
  keisan::Matrix<1, 3> C_d;

  // Gain matrix
  keisan::Matrix<1, 4> F;  // Feedback gain
  std::vector<double> f;   // Preview gain

  // Outputs
  keisan::Matrix<3, 1> x_state;
  keisan::Matrix<3, 1> y_state;
  keisan::Point2 velocity;
  std::vector<COMTrajectory> com_trajectory;
};

}  // namespace gankenkun

#endif  // GANKENKUN__LIPM__LIPM_HPP_
