// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_prodriver_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.hpp"

SimModelIdealSteerAcc::SimModelIdealSteerAcc(float64_t wheelbase)
: SimModelInterface(4 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase)
{
}

float64_t SimModelIdealSteerAcc::getX() { return state_(IDX::X); }
float64_t SimModelIdealSteerAcc::getY() { return state_(IDX::Y); }
float64_t SimModelIdealSteerAcc::getYaw() { return state_(IDX::YAW); }
float64_t SimModelIdealSteerAcc::getVx() { return state_(IDX::VX); }
float64_t SimModelIdealSteerAcc::getVy() { return 0.0; }
float64_t SimModelIdealSteerAcc::getAx() { return input_(IDX_U::AX_DES); }
float64_t SimModelIdealSteerAcc::getWz()
{
  return state_(IDX::VX) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
}
float64_t SimModelIdealSteerAcc::getSteer() { return input_(IDX_U::STEER_DES); }
void SimModelIdealSteerAcc::update(const float64_t & dt) { updateRungeKutta(dt, input_); }

Eigen::VectorXd SimModelIdealSteerAcc::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const float64_t vx = state(IDX::VX);
  const float64_t yaw = state(IDX::YAW);
  const float64_t ax = input(IDX_U::AX_DES);
  const float64_t steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * std::cos(yaw);
  d_state(IDX::Y) = vx * std::sin(yaw);
  d_state(IDX::VX) = ax;
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
}
