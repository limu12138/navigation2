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

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  // CriticManager 负责：
  // - 从参数里读出 critics 列表（字符串）
  // - 通过 pluginlib 动态加载每个 critic 插件
  // - 在每次优化迭代中按顺序调用 critic->scor(data)
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  // pluginlib::ClassLoader 负责从 XML 描述文件中找到派生类并实例化。
  // 这里的 base_class_type 是 mppi::critics::CriticFunction。
  // 重要：critic_names_ 的顺序来自参数 `<controller_namespace>.critics`（YAML 列表顺序保持不变）。
  // 因此：
  // - critics_[0] 对应 YAML 列表的第一个 critic
  // - 也就是“本批 rollout 轨迹评分时，第一个被调用的 scor()”
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    // 参数里写的是短名（例如 "GoalCritic"），这里补全为 C++ 类名。
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  // 评分流程：对同一批 rollout 轨迹，按 critics_ 的顺序叠加成本到 data.costs。
  // critic 可以通过 data.fail_flag 提前中止（例如检测到不可恢复的碰撞）。
  // 首次触发点（按列表顺序）：
  // - 进入本函数后，第一个被调用的必然是 critics_[0]->score(data)
  // - 在默认 bringup 配置中（FollowPath.critics 以 "ConstraintCritic" 开头），
  //   这里第一个被调用的就是 mppi::critics::ConstraintCritic::score()
  //   定义在 nav2_mppi_controller/src/critics/constraint_critic.cpp
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {
      break;
    }
    // 逐个调用每个 critic 的 score() 方法:
    // - ConstraintCritic（约束/可行域惩罚）：超出速度上下界惩罚；Ackermann 还会惩罚
    //   小于最小转弯半径的运动。
    // - CostCritic（costmap 代价/碰撞）：沿轨迹逐点查 costmap；发生碰撞给 collision_cost_。
    //   近障碍区域给较大惩罚，并可能置 data.fail_flag=true（当所有轨迹都碰撞时）。
    // - GoalCritic（到目标位置距离）：仅在“接近目标”时启用，对轨迹到 goal 的平均距离打分。
    // - GoalAngleCritic（到目标朝向角度）：仅在“接近目标”时启用，对 yaw 与 goal yaw 的
    //   平均角差打分。
    // - PathAlignCritic（贴合路径/横向偏差）：远离终点时启用；若局部路径被动态障碍占据
    //   太多，会退出，让障碍相关 critic 接管。
    // - PathFollowCritic（沿路径推进/跟随）：鼓励轨迹末端靠近
    //   “furthest reached point + offset”的路径点；遇到动态障碍会跳到下一个 valid 点。
    // - PathAngleCritic（朝向/指向前方点）：当朝向与“前方路径点方向”夹角过大时惩罚。
    // - PreferForwardCritic（偏好前进）：非接近终点时惩罚 vx<0 的后退运动。
    critics_[q]->score(data);
  }
}

}  // namespace mppi
