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

#include "nav2_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  // NoiseGenerator 做的事情很“纯粹”：
  // - 生成形状为 [batch_size, time_steps] 的高斯噪声张量
  // - 每次迭代把噪声加到当前名义控制序列（control_sequence）上，得到采样控制 state.cvx/cwz/cvy
  // MPPI 的采样空间就是这些 noised controls。
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  if (regenerate_noises_) {
    // 可选：开一个线程提前生成“下一次迭代”要用的噪声，减少主循环耗时。
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    // 默认：在主线程直接生成噪声。
    generateNoisedControls();  // // 为每个维度生成独立高斯噪声
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()  // 触发“下一次噪声”的生成
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  // 将名义控制序列 + 噪声 = 本次迭代要 rollout/评分的 batch 采样控制。
  // 注意：state.cvx/cwz/cvy 的 shape 是 [batch_size, time_steps]。
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Recompute the noises on reset, initialization, and fallback
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls();
  }
}

void NoiseGenerator::noiseThread()
{
  // 后台线程循环：等待主循环发出 ready_ 信号，再生成下一批噪声。
  do {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_;});
    ready_ = false;
    generateNoisedControls();
  } while (active_);
}

void NoiseGenerator::generateNoisedControls()
{
  // 为每个维度生成独立高斯噪声。
  // 这相当于：u_k^i = u_k + eps_k^i，其中 eps ~ N(0, sigma^2)。
  auto & s = settings_;

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
