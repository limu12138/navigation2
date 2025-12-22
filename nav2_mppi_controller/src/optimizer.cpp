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

#include "nav2_mppi_controller/optimizer.hpp"

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace mppi
{

namespace
{

struct ExclusiveViolationCheck
{
  bool lin_ok{false};
  bool ang_ok{false};
  bool violates{false};
};

inline const char * exclusivePolicyToString(const mppi::models::OptimizerSettings & s)
{
  if (s.exclusive_policy == 1) {
    return "LINEAR_PRIORITY";
  }
  if (s.exclusive_policy == 2) {
    return "AUTO";
  }
  return "ANGULAR_PRIORITY";
}

inline ExclusiveViolationCheck checkExclusiveMutualViolation(
  const double vx, const double wz,
  const mppi::models::OptimizerSettings & s)
{
  ExclusiveViolationCheck result;
  result.lin_ok = std::fabs(vx) > s.exclusive_linear_threshold;
  result.ang_ok = std::fabs(wz) > s.exclusive_angular_threshold;
  result.violates = result.lin_ok && result.ang_ok;
  return result;
}

inline void applyExclusiveMutualConstraintScalar(
  float & vx, float & wz,
  const mppi::models::OptimizerSettings & s)
{
  if (!s.exclusive_mode) {
    return;
  }

  const bool lin_ok = std::fabs(vx) > s.exclusive_linear_threshold;
  const bool ang_ok = std::fabs(wz) > s.exclusive_angular_threshold;
  if (!lin_ok && !ang_ok) {
    return;
  }

  // 单轴仅超过阈值：另一轴清零
  if (lin_ok && !ang_ok) {
    wz = 0.0f;
    return;
  }
  if (ang_ok && !lin_ok) {
    vx = 0.0f;
    return;
  }

  // 两轴都超过阈值：按策略选择保留一个轴
  constexpr float eps = 1e-6f;
  const float vx_scale = std::max(
    std::max(std::fabs(s.constraints.vx_max), std::fabs(s.constraints.vx_min)), eps);
  const float wz_scale = std::max(std::fabs(s.constraints.wz), eps);

  const float lin_score = std::fabs(vx) / vx_scale;
  const float ang_score = std::fabs(wz) / wz_scale;

  bool choose_ang = false;
  bool choose_lin = false;
  if (s.exclusive_policy == 0) {  // ANGULAR_PRIORITY
    choose_ang = (ang_score >= lin_score);
    choose_lin = (lin_score > ang_score);
  } else if (s.exclusive_policy == 1) {  // LINEAR_PRIORITY
    choose_lin = (lin_score >= ang_score);
    choose_ang = (ang_score > lin_score);
  } else {  // AUTO (weighted)
    choose_ang = (ang_score * s.exclusive_angular_gain >= lin_score * s.exclusive_linear_gain);
    choose_lin = (lin_score * s.exclusive_linear_gain > ang_score * s.exclusive_angular_gain);
  }

  if (choose_ang) {
    vx = 0.0f;
  }
  if (choose_lin) {
    wz = 0.0f;
  }
}

template<typename TVx, typename TWz>
void applyExclusiveMutualConstraint(
  TVx & vx, TWz & wz,
  const mppi::models::OptimizerSettings & s)
{
  if (!s.exclusive_mode) {
    return;
  }

  // 逐元素施加互斥投影（对整个控制序列 / 整个 batch 控制张量都生效）。
  // 这里避免依赖 xtensor 的逐元素逻辑运算符细节，直接使用标量约束函数，
  // 保证“所有元素都互斥”的语义明确且稳定。
  if (vx.size() != wz.size()) {
    throw std::runtime_error("Exclusive constraint: vx/wz size mismatch");
  }

  auto * vx_ptr = vx.data();
  auto * wz_ptr = wz.data();
  for (size_t i = 0; i < vx.size(); ++i) {
    float vx_scalar = vx_ptr[i];
    float wz_scalar = wz_ptr[i];
    applyExclusiveMutualConstraintScalar(vx_scalar, wz_scalar, s);
    vx_ptr[i] = vx_scalar;
    wz_ptr[i] = wz_scalar;
  }
}

}  // namespace

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.9);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);

  // 新增：读取互斥参数
  getParam(s.exclusive_mode, "exclusive_mode", false);
  getParam(s.exclusive_debug, "exclusive_debug", false);
  std::string policy_str;
  getParam(policy_str, "exclusive_policy", std::string("ANGULAR_PRIORITY"));
  if (policy_str == "LINEAR_PRIORITY") {
    s.exclusive_policy = 1;
  } else if (policy_str == "AUTO") {
    s.exclusive_policy = 2;
  } else {
    s.exclusive_policy = 0;
  }
  getParam(s.exclusive_linear_threshold, "exclusive_linear_threshold", 0.0f);
  getParam(s.exclusive_angular_threshold, "exclusive_angular_threshold", 0.0f);
  // 读取 AUTO 模式的加权比较系数
  getParam(s.exclusive_linear_gain, "exclusive_linear_gain", 1.0f);
  getParam(s.exclusive_angular_gain, "exclusive_angular_gain", 0.3f);

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};

  settings_.constraints = settings_.base_constraints;

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  // 一个控制周期（controller_frequency）内的主入口。
  // 输出是“当前时刻要执行”的 cmd_vel（通常 remap 成 /cmd_vel_nav）。
  // MPPI 做的事情是：
  // - 维护一个名义控制序列 control_sequence_（长度 time_steps）
  // - 采样大量带噪控制序列 rollout 轨迹
  // - critics 对每条轨迹打分得到 costs_
  // - 用 softmax 权重回推更新 control_sequence_
  // 最后从 control_sequence_ 取出 offset 时刻的控制作为本周期输出。
  prepare(robot_pose, robot_speed, plan, goal_checker);  // 首次写入本周期输入数据

  do {
    // optimiz() 内部会跑 iteration_count 次“采样-评分-更新”。
    // 常见设置是 iteration_count=1，通过增大 batch_size 来增强效果。
    optimize();
  } while (fallback(critics_data_.fail_flag));

  // 对 vx / vy / wz 分别做 Savitzky-Golay (二次, 9 点窗口) 滤波
  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  // 平滑后重新施加约束，避免滤波引入线/角同时非零
  // applyControlSequenceConstraints();
  // 取 offset 时刻控制，然后赋值为 ROS TwistStamped
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);

  // Debug: 检查输出控制（controller_server 的 cmd_vel，通常 remap 到 /cmd_vel_nav）
  // 是否满足线/角互斥。
  // 注意：/cmd_vel 最终可能被 velocity_smoother 重新混合；这里检查的是 MPPI 输出链路。
  if (settings_.exclusive_mode) {
    const double vx = control.twist.linear.x;
    const double wz = control.twist.angular.z;
    const auto check = checkExclusiveMutualViolation(vx, wz, settings_);

    if (check.violates) {
      RCLCPP_FATAL(
        logger_,
        "[exclusive] Final, VIOLATION on cmd_vel_nav candidate: vx=%.6f wz=%.6f "
        "(thr_vx=%.6f thr_wz=%.6f policy=%s)",
        vx, wz, settings_.exclusive_linear_threshold, settings_.exclusive_angular_threshold,
        exclusivePolicyToString(settings_));
    } else if (settings_.exclusive_debug) {
      RCLCPP_INFO(
        logger_,
        "[exclusive] Final, cmd_vel_nav ok: vx=%.6f wz=%.6f "
        "(thr_vx=%.6f thr_wz=%.6f policy=%s)",
        vx, wz, settings_.exclusive_linear_threshold, settings_.exclusive_angular_threshold,
        exclusivePolicyToString(settings_));
    }
  }

  if (settings_.shift_control_sequence) {
    shiftControlSequence();  // 名义控制序列向前平移一个时间步
  }

  return control;
}

void Optimizer::optimize()
{
  // MPPI 的一次“优化”由 iteration_count 次迭代组成。
  // 每次迭代固定三步：
  // 1) generateNoisedTrajectorie(): 采样控制 + rollout 轨迹
  // 2) critics.evalTrajectoriesScore(): 轨迹打分，累加到 costs_
  // 3) updateControlSequenc(): softmax 加权更新名义控制序列
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();  // 通过噪声 采样控制 + rollout 轨迹 (运动学传播 + 积分得到轨迹)
    critic_manager_.evalTrajectoriesScores(critics_data_);  // 轨迹打分，累加到 costs_
    updateControlSequence();  // softmax 加权更新名义控制序列, 施加控制约束、互斥、模型约束
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset();

  if (++counter > settings_.retry_attempt_limit) {
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  // 将本周期输入（pose/speed/path）写入内部张量状态。
  // path_ 是把 nav_msgs/Path 转成张量格式，后续 critics 评分与 rollout 都会用到。
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);


  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  // (1) 采样控制：名义 control_sequence_ + 高斯噪声 -> batch_size 条控制序列
  noise_generator_.setNoisedControls(state_, control_sequence_);

  // 互斥约束作为“优化域硬约束”：对采样控制序列做可行域投影
  // 这样生成轨迹、评分、更新与最终输出都在同一可行域中进行。
  // applyExclusiveMutualConstraint(state_.cvx, state_.cwz, settings_);

  noise_generator_.generateNextNoises();  // 触发“下一次噪声”的生成

  // (2) 运动学传播：根据控制序列预测每条轨迹在每个 time step 的速度（state_.vx/wz/vy）
  // - 先把第 0 列速度设置为“当前真实速度”（来自里程计）
  // - 再将控制序列右移一格，得到后续 time step 的预测速度
  updateStateVelocities(state_);

  // (3) 积分得到轨迹：把速度积分成 (x,y,yaw) 序列，供 critics 批量打分
  // 输出：generated_trajectories_ 内的 (x, y, yaw) 序列
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  // 对名义控制序列做硬约束裁剪（速度边界）。
  // 注意：后面还会调用 motion_model_->applyConstraints()（例如 Ackermann 的额外约束）。
  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }
  // 裁剪线速度
  control_sequence_.vx = xt::clip(
    control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  // 裁剪角速度
  control_sequence_.wz = xt::clip(
    control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  motion_model_->applyConstraints(control_sequence_);  // 应用运动模型特定约束

  // 约束阶段施加线/角互斥（作用于整条控制序列）
  // applyExclusiveMutualConstraint(control_sequence_.vx, control_sequence_.wz, s);
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  // 将 state.{vx,wz[,vy]} 的第 0 列设置为“当前真实速度”（来自里程计）。
  updateInitialStateVelocities(state);
  // 将当前控制输入向右平移一个时间步作为下一时刻的预测速度，得到整段预测速度序列。
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  // 将 state.vx 的第 0 列设置为当前线速度（x）。
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  // 将单条控制序列（time_steps x {vx,wz[,vy]}）积分成单条轨迹（time_steps x {x,y,yaw}）。
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  // 获取初始航向角
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  xt::noalias(trajectories.yaws) =
    xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;  // 累积角速度得到航向角序列

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  // 创建航向角余弦/正弦张量
  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  // 设置初始航向角的余弦/正弦值
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  // 计算后续航向角的余弦/正弦值
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) =
    xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) =
    xt::sin(yaws_cutted);

  auto && dx = xt::eval(state.vx * yaw_cos);  // 计算每个时间步的 x 方向增量
  auto && dy = xt::eval(state.vx * yaw_sin);  // 计算每个时间步的 y 方向增量

  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    xt::cumsum(dx * settings_.model_dt, 1);  // 积分得到 x 坐标序列
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    xt::cumsum(dy * settings_.model_dt, 1);  // 积分得到 y 坐标序列
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  auto && sequence =
    xt::xtensor<float, 2>::from_shape(
    {settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  // MPPI 的更新步骤：
  // - costs_ 已经由 critics 累加了每条采样轨迹的代价 J_i
  // - 这里叠加一个“控制代价”项（与采样噪声相关），再做 softmax
  // - 用 softmax 权重对采样控制序列做加权平均，得到新的名义 control_sequence_
  auto & s = settings_;
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  // 控制代价项（与采样噪声相关）
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx,
    1, immediate);
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);  // 减去最小值以防溢出
  // temperature 越小越“偏向最优样本”，越大越“平均/平滑”。
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));  // 计算 softmax 权重
  // 扩展维度以便广播
  auto && softmaxes_extened = xt::eval(
    xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) =
    xt::sum(state_.cvx * softmaxes_extened, 0, immediate);  // 加权平均更新名义控制序列
  xt::noalias(control_sequence_.wz) =
    xt::sum(state_.cwz * softmaxes_extened, 0, immediate);  // 更新名义控制序列
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) =
      xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  applyControlSequenceConstraints();  // 施加控制约束、互斥、模型约束

  // Debug: 互斥检查要在标量 vx/wz 上做（这里取本周期会输出的 offset 元素）。
  if (settings_.exclusive_mode && settings_.exclusive_debug) {
    const unsigned int offset = settings_.shift_control_sequence ? 1 : 0;
    const float vx = control_sequence_.vx(offset);
    const float wz = control_sequence_.wz(offset);
    const auto check = checkExclusiveMutualViolation(vx, wz, settings_);
    if (check.violates) {
      RCLCPP_ERROR(
        logger_,
        "[exclusive] 1, violent cmd_vel_nav: raw(vx=%.6f,wz=%.6f)",
        static_cast<double>(vx), static_cast<double>(wz));
    }
  }
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  // 从名义控制序列取出“本周期要执行”的控制。
  // 若 shift_control_sequence=true，则 offset=1 表示跳过第 0 个，避免重复输出相同控制。
  unsigned int offset = settings_.shift_control_sequence ? 1 : 0;

  float vx = control_sequence_.vx(offset);
  float wz = control_sequence_.wz(offset);

  // const float vx_raw = vx;
  // const float wz_raw = wz;
  // applyExclusiveMutualConstraintScalar(vx, wz, settings_);

  // if (settings_.exclusive_debug)
  // {
  //   const auto raw_check = checkExclusiveMutualViolation(vx_raw, wz_raw, settings_);
  //   if (raw_check.violates)
  //   {
  //     RCLCPP_ERROR(
  //         logger_,
  //         "[exclusive] before changing, violent cmd_vel_nav: "
  //         "raw(vx=%.6f,wz=%.6f) -> proj(vx=%.6f,wz=%.6f)",
  //         vx_raw, wz_raw, vx, wz);
  //   }

  //   const bool changed = (vx_raw != vx) || (wz_raw != wz);
  //   if (changed)
  //   {
  //     RCLCPP_INFO(
  //         logger_,
  //         "[exclusive] final projection changed cmd_vel_nav!");
  //   }
  // }

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());  // 赋值ros消息
}

void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_, name_);
  } else {
    throw std::runtime_error(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace mppi
