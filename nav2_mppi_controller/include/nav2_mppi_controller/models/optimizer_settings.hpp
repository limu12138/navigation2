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

#ifndef NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
#define NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_

#include <cstddef>
#include "nav2_mppi_controller/models/constraints.hpp"

namespace mppi::models
{

/**
 * @struct mppi::models::OptimizerSettings
 * @brief Settings for the optimizer to use
 */
struct OptimizerSettings
{
  models::ControlConstraints base_constraints{0, 0, 0, 0};
  models::ControlConstraints constraints{0, 0, 0, 0};
  models::SamplingStd sampling_std{0, 0, 0};
  float model_dt{0};
  float temperature{0};
  float gamma{0};
  unsigned int batch_size{0};
  unsigned int time_steps{0};
  unsigned int iteration_count{0};
  bool shift_control_sequence{false};
  size_t retry_attempt_limit{0};

  // 新增：线/角互斥相关配置
  bool exclusive_mode{false};
  bool exclusive_debug{false};
  int exclusive_policy{0};                // 0: ANGULAR_PRIORITY, 1: LINEAR_PRIORITY, 2: AUTO
  float exclusive_linear_threshold{0.0f};
  float exclusive_angular_threshold{0.0f};
  // 加权比较（AUTO 模式使用）：线速度权重与角速度折算系数
  float exclusive_linear_gain{1.0f};
  float exclusive_angular_gain{0.3f};
};

}  // namespace mppi::models

#endif  // NAV2_MPPI_CONTROLLER__MODELS__OPTIMIZER_SETTINGS_HPP_
