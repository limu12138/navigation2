// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING

#include "nav2_navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_navfn_planner
{

NavfnPlanner::NavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type NavfnPlanner",
    name_.c_str());
}

void
NavfnPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  node_ = parent;
  auto node = parent.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "Configuring plugin %s of type NavfnPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_astar", use_astar_);
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

  // Polyline post-processing parameters
  declare_parameter_if_not_declared(
    node, name + ".polyline_enable_shortcut", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".polyline_enable_shortcut", polyline_enable_shortcut_);
  declare_parameter_if_not_declared(
    node, name + ".polyline_resample_step", rclcpp::ParameterValue(0.0));
  node->get_parameter(name + ".polyline_resample_step", polyline_resample_step_);
  declare_parameter_if_not_declared(
    node, name + ".polyline_max_cost", rclcpp::ParameterValue(252));
  node->get_parameter(name + ".polyline_max_cost", polyline_max_cost_);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<NavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());
}

void
NavfnPlanner::activate()
{
  RCLCPP_INFO(
    logger_, "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&NavfnPlanner::dynamicParametersCallback, this, _1));
}

void
NavfnPlanner::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
  dyn_params_handler_.reset();
}

void
NavfnPlanner::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type NavfnPlanner",
    name_.c_str());
  planner_.reset();
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  if (start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y)
  {
    unsigned int mx, my;
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
    if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
      return path;
    }
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
    return path;
  }

  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    RCLCPP_WARN(
      logger_, "%s: failed to create plan with "
      "tolerance %.2f.", name_.c_str(), tolerance_);
  }


#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  std::cout << "It took " << time_span.count() * 1000 << std::endl;
#endif

  return path;
}

bool
NavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

bool
NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "Cannot create a plan: the robot's start position is off the global"
      " costmap. Planning will always fail, are you sure"
      " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  lock.unlock();

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "The goal sent to the planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p = goal;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);

      // Post-process path into a more polyline-like path (fewer turns / longer straight segments)
      postProcessPathToPolyline(plan);

      // Ensure the plan starts at the robot pose to improve local controller tracking.
      // Some controllers (e.g., DWB) behave poorly if the first path pose is not close to the robot.
      if (!plan.poses.empty()) {
        const double resolution = costmap_->getResolution();
        const double dx = plan.poses.front().pose.position.x - start.position.x;
        const double dy = plan.poses.front().pose.position.y - start.position.y;
        if ((dx * dx + dy * dy) > (resolution * resolution)) {
          geometry_msgs::msg::PoseStamped start_pose;
          start_pose.header = plan.header;
          start_pose.pose = start;
          plan.poses.insert(plan.poses.begin(), start_pose);
        }
      }

      // If use_final_approach_orientation=true, interpolate the last pose orientation from the
      // previous pose to set the orientation to the 'final approach' orientation of the robot so
      // it does not rotate.
      // And deal with corner case of plan of length 1
      if (use_final_approach_orientation_) {
        size_t plan_size = plan.poses.size();
        if (plan_size == 1) {
          plan.poses.back().pose.orientation = start.orientation;
        } else if (plan_size > 1) {
          double dx, dy, theta;
          auto last_pose = plan.poses.back().pose.position;
          auto approach_pose = plan.poses[plan_size - 2].pose.position;
          // Deal with the case of NavFn producing a path with two equal last poses
          if (std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
            std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2)
          {
            approach_pose = plan.poses[plan_size - 3].pose.position;
          }
          
          dx = last_pose.x - approach_pose.x;
          dy = last_pose.y - approach_pose.y;
          theta = atan2(dy, dx);
          plan.poses.back().pose.orientation =
            nav2_util::geometry_utils::orientationAroundZAxis(theta);
        }
      }
    } else {
      RCLCPP_ERROR(
        logger_,
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }

  return !plan.poses.empty();
}

void
NavfnPlanner::postProcessPathToPolyline(nav_msgs::msg::Path & plan)
{
  if (plan.poses.size() < 3) {
    return;
  }

  auto set_headers = [&plan]() {
      for (auto & pose : plan.poses) {
        pose.header = plan.header;
      }
    };

  set_headers();

  // Remove near-duplicate consecutive points
  {
    constexpr double kEps = 1e-6;
    std::vector<geometry_msgs::msg::PoseStamped> dedup;
    dedup.reserve(plan.poses.size());
    dedup.push_back(plan.poses.front());
    for (size_t i = 1; i < plan.poses.size(); ++i) {
      const auto & prev = dedup.back().pose.position;
      const auto & curr = plan.poses[i].pose.position;
      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      if ((dx * dx + dy * dy) > kEps) {
        dedup.push_back(plan.poses[i]);
      }
    }
    plan.poses.swap(dedup);
    set_headers();
  }

  if (plan.poses.size() < 3) {
    return;
  }

  // Remove nearly-collinear middle points (keeps corners)
  {
    constexpr double kCrossEps = 1e-6;
    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    pruned.reserve(plan.poses.size());
    pruned.push_back(plan.poses.front());

    for (size_t i = 1; i + 1 < plan.poses.size(); ++i) {
      const auto & a = pruned.back().pose.position;
      const auto & b = plan.poses[i].pose.position;
      const auto & c = plan.poses[i + 1].pose.position;

      const double abx = b.x - a.x;
      const double aby = b.y - a.y;
      const double bcx = c.x - b.x;
      const double bcy = c.y - b.y;
      const double cross = abx * bcy - aby * bcx;

      if (std::abs(cross) > kCrossEps) {
        pruned.push_back(plan.poses[i]);
      }
    }

    pruned.push_back(plan.poses.back());
    plan.poses.swap(pruned);
    set_headers();
  }

  if (plan.poses.size() < 3) {
    return;
  }

  // Line-of-sight shortcutting on costmap grid (Bresenham)
  // This greedily connects as far as possible with collision-free straight segments.
  if (polyline_enable_shortcut_) {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

    auto is_cell_free = [this](int mx, int my) -> bool {
        if (mx < 0 || my < 0) {
          return false;
        }
        if (mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
          my >= static_cast<int>(costmap_->getSizeInCellsY()))
        {
          return false;
        }
        const unsigned char cost = costmap_->getCost(static_cast<unsigned int>(mx),
            static_cast<unsigned int>(my));
        if (!allow_unknown_ && cost == nav2_costmap_2d::NO_INFORMATION) {
          return false;
        }

        // Keep a safety margin: reject high-cost inflated cells as well.
        // polyline_max_cost_ defaults to 252 (just below INSCRIBED=253).
        if (cost >= static_cast<unsigned char>(polyline_max_cost_)) {
          return false;
        }
        return true;
      };

    auto is_segment_free = [this, &is_cell_free](
      const geometry_msgs::msg::PoseStamped & from,
      const geometry_msgs::msg::PoseStamped & to) -> bool
      {
        unsigned int x0_u, y0_u, x1_u, y1_u;
        if (!worldToMap(from.pose.position.x, from.pose.position.y, x0_u, y0_u)) {
          return false;
        }
        if (!worldToMap(to.pose.position.x, to.pose.position.y, x1_u, y1_u)) {
          return false;
        }

        int x0 = static_cast<int>(x0_u);
        int y0 = static_cast<int>(y0_u);
        int x1 = static_cast<int>(x1_u);
        int y1 = static_cast<int>(y1_u);

        const int dx = std::abs(x1 - x0);
        const int dy = std::abs(y1 - y0);
        const int sx = (x0 < x1) ? 1 : -1;
        const int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int x = x0;
        int y = y0;

        while (true) {
          if (!is_cell_free(x, y)) {
            return false;
          }
          if (x == x1 && y == y1) {
            break;
          }
          const int e2 = 2 * err;
          if (e2 > -dy) {
            err -= dy;
            x += sx;
          }
          if (e2 < dx) {
            err += dx;
            y += sy;
          }
        }
        return true;
      };

    std::vector<geometry_msgs::msg::PoseStamped> simplified;
    simplified.reserve(plan.poses.size());
    simplified.push_back(plan.poses.front());

    const size_t n = plan.poses.size();
    size_t i = 0;
    size_t j = 1;
    size_t last_good = 1;

    while (j < n) {
      if (is_segment_free(plan.poses[i], plan.poses[j])) {
        last_good = j;
        ++j;
      } else {
        // Commit the farthest visible point and restart from there.
        simplified.push_back(plan.poses[last_good]);
        i = last_good;
        j = i + 1;
        last_good = j;
        if (j >= n) {
          break;
        }
      }
    }

    if (simplified.empty() || simplified.back().pose.position != plan.poses.back().pose.position) {
      simplified.push_back(plan.poses.back());
    }

    plan.poses.swap(simplified);
    set_headers();
  }

  // Final collinear prune after shortcutting (optional but keeps it clean)
  if (plan.poses.size() >= 3) {
    constexpr double kCrossEps = 1e-6;
    std::vector<geometry_msgs::msg::PoseStamped> pruned;
    pruned.reserve(plan.poses.size());
    pruned.push_back(plan.poses.front());

    for (size_t i = 1; i + 1 < plan.poses.size(); ++i) {
      const auto & a = pruned.back().pose.position;
      const auto & b = plan.poses[i].pose.position;
      const auto & c = plan.poses[i + 1].pose.position;

      const double abx = b.x - a.x;
      const double aby = b.y - a.y;
      const double bcx = c.x - b.x;
      const double bcy = c.y - b.y;
      const double cross = abx * bcy - aby * bcx;

      if (std::abs(cross) > kCrossEps) {
        pruned.push_back(plan.poses[i]);
      }
    }

    pruned.push_back(plan.poses.back());
    plan.poses.swap(pruned);
    for (auto & pose : plan.poses) {
      pose.header = plan.header;
    }
  }

  // Recompute orientations to follow the path tangent.
  // Many local planners/controllers use the path pose orientation for heading error.
  if (plan.poses.size() >= 2) {
    for (size_t i = 0; i + 1 < plan.poses.size(); ++i) {
      const auto & p0 = plan.poses[i].pose.position;
      const auto & p1 = plan.poses[i + 1].pose.position;
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) {
        continue;
      }
      const double theta = std::atan2(dy, dx);
      plan.poses[i].pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }

    // Set the last pose orientation to match the incoming segment.
    // (If use_final_approach_orientation_ is enabled, NavfnPlanner will override it later.)
    plan.poses.back().pose.orientation = plan.poses.end()[-2].pose.orientation;
  }

  // Densify the polyline with evenly spaced points along each segment.
  // This keeps the polyline shape but improves local planner critics that assume a reasonably
  // dense global plan (e.g., DWB PathAlign/PathDist).
  if (plan.poses.size() >= 2) {
    double step = polyline_resample_step_;
    if (step <= 0.0) {
      step = costmap_->getResolution();
    }
    step = std::max(0.01, step);
    std::vector<geometry_msgs::msg::PoseStamped> resampled;
    resampled.reserve(plan.poses.size());
    resampled.push_back(plan.poses.front());

    for (size_t i = 0; i + 1 < plan.poses.size(); ++i) {
      const auto & a = plan.poses[i].pose.position;
      const auto & b = plan.poses[i + 1].pose.position;
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double dist = std::hypot(dx, dy);
      if (dist < 1e-9) {
        continue;
      }

      const double theta = std::atan2(dy, dx);
      const auto q = nav2_util::geometry_utils::orientationAroundZAxis(theta);

      const int num_steps = static_cast<int>(std::floor(dist / step));
      for (int s = 1; s <= num_steps; ++s) {
        const double t = std::min(1.0, (s * step) / dist);
        geometry_msgs::msg::PoseStamped p;
        p.header = plan.header;
        p.pose.position.x = a.x + t * dx;
        p.pose.position.y = a.y + t * dy;
        p.pose.position.z = 0.0;
        p.pose.orientation = q;
        resampled.push_back(p);
      }

      // Ensure we include the exact segment end.
      geometry_msgs::msg::PoseStamped end = plan.poses[i + 1];
      end.header = plan.header;
      end.pose.orientation = q;
      resampled.push_back(end);
    }

    // Remove any accidental duplicates after resampling.
    {
      constexpr double kEps = 1e-10;
      std::vector<geometry_msgs::msg::PoseStamped> dedup;
      dedup.reserve(resampled.size());
      dedup.push_back(resampled.front());
      for (size_t i = 1; i < resampled.size(); ++i) {
        const auto & prev = dedup.back().pose.position;
        const auto & curr = resampled[i].pose.position;
        const double ddx = curr.x - prev.x;
        const double ddy = curr.y - prev.y;
        if ((ddx * ddx + ddy * ddy) > kEps) {
          dedup.push_back(resampled[i]);
        }
      }
      plan.poses.swap(dedup);
    }
  }
}

void
NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  goal_copy.header = plan.header;
  plan.poses.push_back(goal_copy);
}

bool
NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(
    logger_,
    "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

// bool
// NavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
// {
//   return validPointPotential(world_point, tolerance_);
// }

// bool
// NavfnPlanner::validPointPotential(
//   const geometry_msgs::msg::Point & world_point, double tolerance)
// {
//   const double resolution = costmap_->getResolution();

//   geometry_msgs::msg::Point p = world_point;
//   double potential = getPointPotential(p);
//   if (potential < POT_HIGH) {
//     // world_point is reachable by itself
//     return true;
//   } else {
//     // world_point, is not reachable. Trying to find any
//     // reachable point within its tolerance region
//     p.y = world_point.y - tolerance;
//     while (p.y <= world_point.y + tolerance) {
//       p.x = world_point.x - tolerance;
//       while (p.x <= world_point.x + tolerance) {
//         potential = getPointPotential(p);
//         if (potential < POT_HIGH) {
//           return true;
//         }
//         p.x += resolution;
//       }
//       p.y += resolution;
//     }
//   }

//   return false;
// }

bool
NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_,
    "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

rcl_interfaces::msg::SetParametersResult
NavfnPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = parameter.as_double();
      } else if (name == name_ + ".polyline_resample_step") {
        polyline_resample_step_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == name_ + ".polyline_max_cost") {
        polyline_max_cost_ = parameter.as_int();
        if (polyline_max_cost_ < 0) {
          polyline_max_cost_ = 0;
        } else if (polyline_max_cost_ > 255) {
          polyline_max_cost_ = 255;
        }
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = parameter.as_bool();
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = parameter.as_bool();
      } else if (name == name_ + ".use_final_approach_orientation") {
        use_final_approach_orientation_ = parameter.as_bool();
      } else if (name == name_ + ".polyline_enable_shortcut") {
        polyline_enable_shortcut_ = parameter.as_bool();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_navfn_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_navfn_planner::NavfnPlanner, nav2_core::GlobalPlanner)
