// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include "nav2_mppi_controller/critics/constraint_critic.hpp"

#include <string>

#include <xtensor/xoperation.hpp>

namespace mppi::critics
{

void ConstraintCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 4.0);

  // Soft mutual-exclusivity (vx/wz) penalty parameters.
  // If exclusive_weight_ <= 0, this soft-constraint is effectively disabled.
  getParam(exclusive_linear_epsilon_, "exclusive_linear_epsilon", 0.0);
  getParam(exclusive_angular_epsilon_, "exclusive_angular_epsilon", 0.0);
  getParam(exclusive_weight_, "exclusive_cost_weight", 0.0);
  getParam(exclusive_power_, "exclusive_cost_power", 1);

  std::string exclusive_cost_mode_str;
  getParam(exclusive_cost_mode_str, "exclusive_cost_mode", std::string("product"));
  if (exclusive_cost_mode_str == "indicator") {
    exclusive_cost_mode_ = 1U;
  } else if (exclusive_cost_mode_str == "min") {
    exclusive_cost_mode_ = 2U;
  } else {
    exclusive_cost_mode_ = 0U;
  }

  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  if (exclusive_weight_ > 0.0F) {
    RCLCPP_INFO(
      logger_,
      "ConstraintCritic mutual-exclusivity soft penalty enabled: "
      "mode=%s eps_v=%f eps_w=%f weight=%f power=%u",
      exclusive_cost_mode_str.c_str(),
      exclusive_linear_epsilon_,
      exclusive_angular_epsilon_,
      exclusive_weight_,
      exclusive_power_);
  }

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min, "vx_min", -0.35);

  const float min_sgn = vx_min > 0.0 ? 1.0 : -1.0;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  // 首次触发点（默认配置）：
  // - CriticManager::evalTrajectoriesScores() 会按参数 `FollowPath.critics: [...]` 的顺序
  //   逐个调用 critic->score(data)。
  // - 在默认 bringup 参数里，第一个 critic 通常就是 "ConstraintCritic"，因此这里往往是
  //   “每次 MPPI 迭代中，第一个被调用的 score()”。
  //
  // 本 critic 的职责：对 rollout 轨迹施加“违反速度/几何约束”的惩罚。
  // - 输入：data.state.{vx,vy,wz} 是 (batch_size x time_steps) 的张量，表示每条采样
  //   轨迹在每个时刻的速度。
  // - 输出：把惩罚按时间积分（乘以 data.model_dt 后在 time_steps 上求和）并累加到 data.costs。
  //   data.costs 是长度 batch_size 的向量：每条轨迹一个总代价。

  if (!enabled_) {
    return;
  }

  auto sgn = xt::where(data.state.vx > 0.0, 1.0, -1.0);
  auto vel_total = sgn *
    xt::sqrt(data.state.vx * data.state.vx + data.state.vy * data.state.vy);
  auto out_of_max_bounds_motion = xt::maximum(vel_total - max_vel_, 0);
  auto out_of_min_bounds_motion = xt::maximum(min_vel_ - vel_total, 0);

  // Soft mutual-exclusivity penalty: add a large cost where both |vx| and |wz|
  // exceed their epsilons at the same timestep.
  //
  // We use an "excess" formulation so the penalty is 0 until thresholds are
  // crossed, then grows with both magnitudes:
  //   violation = max(|vx|-eps_v, 0) * max(|wz|-eps_w, 0)
  //
  // Shape: (batch_size x time_steps)
  const bool exclusive_enabled =
    (exclusive_weight_ > 0.0F) &&
    ((exclusive_linear_epsilon_ > 0.0F) || (exclusive_angular_epsilon_ > 0.0F));
  if (exclusive_enabled) {
    const float eps_v = exclusive_linear_epsilon_;
    const float eps_w = exclusive_angular_epsilon_;
    auto v_excess = xt::maximum(xt::fabs(data.state.vx) - eps_v, 0.0F);
    auto w_excess = xt::maximum(xt::fabs(data.state.wz) - eps_w, 0.0F);

    // NOTE:
    // - product: penalize proportional to both magnitudes (smooth, but may
    //   encourage shrinking both vx and wz if weight is huge / other critics
    //   conflict)
    // - indicator: pure boolean penalty (closer to "no simultaneous vx/wz";
    //   doesn't reward shrinking magnitudes once violating)
    // - min: penalize the "minor axis" excess (encourages keeping one axis and
    //   zeroing the other, reducing chattering)
    xt::xtensor<float, 2> exclusive_violation = xt::eval(v_excess * w_excess);
    if (exclusive_cost_mode_ == 1U) {
      auto mask =
        (xt::fabs(data.state.vx) > eps_v) &&
        (xt::fabs(data.state.wz) > eps_w);
      exclusive_violation = xt::eval(xt::where(mask, 1.0F, 0.0F));
    } else if (exclusive_cost_mode_ == 2U) {
      exclusive_violation = xt::eval(xt::minimum(v_excess, w_excess));
    }

    data.costs += xt::pow(
      xt::sum(std::move(exclusive_violation) * data.model_dt, {1}, immediate) *
      exclusive_weight_,
      exclusive_power_);
  }

  auto acker = dynamic_cast<AckermannMotionModel *>(data.motion_model.get());
  if (acker != nullptr) {
    auto & vx = data.state.vx;
    auto & wz = data.state.wz;
    auto out_of_turning_rad_motion = xt::maximum(
      acker->getMinTurningRadius() - (xt::fabs(vx) / xt::fabs(wz)), 0.0);

    data.costs += xt::pow(
      xt::sum(
        (std::move(out_of_max_bounds_motion) +
        std::move(out_of_min_bounds_motion) +
        std::move(out_of_turning_rad_motion)) *
        data.model_dt, {1}, immediate) * weight_, power_);
    return;
  }

  data.costs += xt::pow(
    xt::sum(
      (std::move(out_of_max_bounds_motion) +
      std::move(out_of_min_bounds_motion)) *
      data.model_dt, {1}, immediate) * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::ConstraintCritic, mppi::critics::CriticFunction)
