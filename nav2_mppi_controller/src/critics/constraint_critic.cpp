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

#include <algorithm>
#include <string>

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

  // Additional optional forms (keep product too).
  getParam(exclusive_min_weight_, "exclusive_min_cost_weight", 0.0);
  getParam(exclusive_min_power_, "exclusive_min_cost_power", 1);
  getParam(exclusive_case_weight_, "exclusive_case_cost_weight", 0.0);
  getParam(exclusive_case_power_, "exclusive_case_cost_power", 1);
  getParam(exclusive_switch_weight_, "exclusive_switch_cost_weight", 0.0);
  getParam(exclusive_switch_power_, "exclusive_switch_cost_power", 1);

  RCLCPP_INFO(
    logger_, "ConstraintCritic instantiated with %d power and %f weight.",
    power_, weight_);

  if (exclusive_weight_ > 0.0F) {
    RCLCPP_INFO(
      logger_,
      "ConstraintCritic mutual-exclusivity soft penalty enabled: "
      "eps_v=%f eps_w=%f weight=%f power=%u",
      exclusive_linear_epsilon_,
      exclusive_angular_epsilon_,
      exclusive_weight_,
      exclusive_power_);
  }

  float vy_max;
  getParentParam(vx_max_, "vx_max", 0.5);
  getParentParam(vy_max, "vy_max", 0.0);
  getParentParam(vx_min_, "vx_min", -0.35);
  getParentParam(wz_max_, "wz_max", 1.9);

  const float min_sgn = vx_min_ > 0.0F ? 1.0F : -1.0F;
  max_vel_ = sqrtf(vx_max_ * vx_max_ + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min_ * vx_min_ + vy_max * vy_max);

  // Reuse the optimizer's exclusive policy / gains if present under parent.
  // Parent param uses a string in many configs; map to int.
  std::string policy_str;
  getParentParam(policy_str, "exclusive_policy", std::string("AUTO"));
  if (policy_str == "ANGULAR_PRIORITY") {
    exclusive_policy_ = 0;
  } else if (policy_str == "LINEAR_PRIORITY") {
    exclusive_policy_ = 1;
  } else {
    exclusive_policy_ = 2;
  }
  getParentParam(exclusive_linear_gain_, "exclusive_linear_gain", 1.0F);
  getParentParam(exclusive_angular_gain_, "exclusive_angular_gain", 1.0F);
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

  // Soft mutual-exclusivity penalties (walk-then-turn-then-walk).
  // These are optional and can be combined:
  //  1) Product (keep): v_excess * w_excess
  //  2) Min (continuous): min(v_excess/v_scale, w_excess/w_scale)
  //  3) Case/policy: if both exceed, penalize the axis to be suppressed
  const bool any_exclusive_enabled =
    (exclusive_weight_ > 0.0F) || (exclusive_min_weight_ > 0.0F) ||
    (exclusive_case_weight_ > 0.0F) || (exclusive_switch_weight_ > 0.0F);
  const bool exclusive_thresholds_enabled =
    (exclusive_linear_epsilon_ > 0.0F) || (exclusive_angular_epsilon_ > 0.0F);

  if (any_exclusive_enabled && exclusive_thresholds_enabled) {
    constexpr float eps = 1e-6F;
    const float eps_v = exclusive_linear_epsilon_;
    const float eps_w = exclusive_angular_epsilon_;

    const float vx_scale = std::max(
      std::max(std::fabs(vx_max_), std::fabs(vx_min_)), eps);
    const float wz_scale = std::max(std::fabs(wz_max_), eps);

    auto v_abs = xt::fabs(data.state.vx);
    auto w_abs = xt::fabs(data.state.wz);
    auto v_excess = xt::maximum(v_abs - eps_v, 0.0F);
    auto w_excess = xt::maximum(w_abs - eps_w, 0.0F);
    auto both_over = (v_excess > 0.0F) & (w_excess > 0.0F);

    if (exclusive_weight_ > 0.0F) {
      auto exclusive_violation = v_excess * w_excess;
      data.costs += xt::pow(
        xt::sum(std::move(exclusive_violation) * data.model_dt, {1}, immediate) *
        exclusive_weight_,
        exclusive_power_);
    }

    if (exclusive_min_weight_ > 0.0F) {
      auto v_norm = v_excess / vx_scale;
      auto w_norm = w_excess / wz_scale;
      auto min_violation = xt::minimum(v_norm, w_norm);
      data.costs += xt::pow(
        xt::sum(std::move(min_violation) * data.model_dt, {1}, immediate) *
        exclusive_min_weight_,
        exclusive_min_power_);
    }

    if (exclusive_case_weight_ > 0.0F) {
      auto lin_score = v_abs / vx_scale;
      auto ang_score = w_abs / wz_scale;

      xt::xarray<bool> choose_ang;
      xt::xarray<bool> choose_lin;
      if (exclusive_policy_ == 0) {  // ANGULAR_PRIORITY
        choose_ang = (ang_score >= lin_score);
        choose_lin = (lin_score > ang_score);
      } else if (exclusive_policy_ == 1) {  // LINEAR_PRIORITY
        choose_lin = (lin_score >= ang_score);
        choose_ang = (ang_score > lin_score);
      } else {  // AUTO (weighted)
        choose_ang =
          (ang_score * exclusive_angular_gain_ >= lin_score * exclusive_linear_gain_);
        choose_lin =
          (lin_score * exclusive_linear_gain_ > ang_score * exclusive_angular_gain_);
      }

      // Use full normalized magnitudes (not only the excess) so this term has
      // a meaningful effect with moderate weights, while still gating on
      // both_over to avoid fighting normal single-mode motion.
      auto suppress_v = xt::where(both_over & choose_ang, lin_score, 0.0F);
      auto suppress_w = xt::where(both_over & choose_lin, ang_score, 0.0F);
      auto case_violation = suppress_v + suppress_w;

      data.costs += xt::pow(
        xt::sum(std::move(case_violation) * data.model_dt, {1}, immediate) *
        exclusive_case_weight_,
        exclusive_case_power_);
    }

    // Switching penalty: discourage rapid alternation between lin and ang.
    // This helps reduce "twitching" when strict mutual exclusivity is active.
    //
    // switch = (lin_t & ang_{t-1}) | (ang_t & lin_{t-1})
    if (exclusive_switch_weight_ > 0.0F) {
      auto lin_on = (v_abs > eps_v);
      auto ang_on = (w_abs > eps_w);
      auto lin_prev = xt::view(lin_on, xt::all(), xt::range(0, -1));
      auto ang_prev = xt::view(ang_on, xt::all(), xt::range(0, -1));
      auto lin_next = xt::view(lin_on, xt::all(), xt::range(1, xt::placeholders::_));
      auto ang_next = xt::view(ang_on, xt::all(), xt::range(1, xt::placeholders::_));
      auto switch_mask = (lin_next & ang_prev) | (ang_next & lin_prev);
      auto switch_cost = xt::where(switch_mask, 1.0F, 0.0F);

      data.costs += xt::pow(
        xt::sum(std::move(switch_cost) * data.model_dt, {1}, immediate) *
        exclusive_switch_weight_,
        exclusive_switch_power_);
    }
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
