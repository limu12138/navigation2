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

#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for enforcing feasible constraints
 */
class ConstraintCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

  float getMaxVelConstraint() {return max_vel_;}
  float getMinVelConstraint() {return min_vel_;}

protected:
  unsigned int power_{0};
  float weight_{0};
  float min_vel_;
  float max_vel_;

  // Parent constraints cached for normalization / scoring.
  float vx_max_{0.5F};
  float vx_min_{-0.35F};
  float wz_max_{1.9F};

  // Soft mutual-exclusivity constraint (walk-then-turn-then-walk):
  // penalize timesteps where both |vx| and |wz| exceed their epsilons.
  float exclusive_linear_epsilon_{0.0F};
  float exclusive_angular_epsilon_{0.0F};

  // Product penalty: max(|vx|-eps_v,0) * max(|wz|-eps_w,0)
  float exclusive_weight_{0.0F};
  unsigned int exclusive_power_{1U};

  // Min penalty (continuous alternative): min(v_excess/v_scale, w_excess/w_scale)
  float exclusive_min_weight_{0.0F};
  unsigned int exclusive_min_power_{1U};

  // Policy-based penalty: when both exceed thresholds, penalize the axis
  // that the policy says should be suppressed (mimics hard projection).
  float exclusive_case_weight_{0.0F};
  unsigned int exclusive_case_power_{1U};

  // Switch penalty: penalize frequent toggling between linear and angular
  // modes across consecutive timesteps.
  float exclusive_switch_weight_{0.0F};
  unsigned int exclusive_switch_power_{1U};

  // Reuse MPPI optimizer's policy/gains for consistent behavior.
  // 0: ANGULAR_PRIORITY, 1: LINEAR_PRIORITY, 2: AUTO
  int exclusive_policy_{2};
  float exclusive_linear_gain_{1.0F};
  float exclusive_angular_gain_{1.0F};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CONSTRAINT_CRITIC_HPP_
