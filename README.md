# README.md
## 一. 项目作用
作为vln项目中全局路径搜索+路径优化+轨迹跟踪+局部避障的部分


[INPUT]gird_map可通行图层, 目标导航点nav_goal
[OUTPUT]小车cmd_vel的控制序列



- [TODO] 地图接收相关
  - [TODO]把地图适配到map_generator这个仓库发布的grid_map的可通行图层上
  - [TODO]在接收到了可通行图层之后，自己能够识别动态障碍物加到可通行图层上
- [TODO] 定位
  - [TODO]定位上暂时不使用acml，拿一个gazebo的odom直接作为位置反馈(实物上使用fast_lio2或者point_lio来拿自己的位置)
- [TODO] 控制
  - [TODO]支持使用vln_gazebo_simulator中的小车的仿真接口
- [TODO]局部避障验证与实现
  - [TODO]需要在vln_gazebo_simulator中增加局部障碍物(Tipriest加)
 
## 项目使用
```
git submodule update --init --recursive
```

## 编译流程
```
colcon build --packages-select map_generator
colcon build --packages-up-to navigation2 --symlink-install
colcon build --packages-up-to nav2_bringup --symlink-install --allow-overriding nav2_map_server nav2_msgs
<!-- colcon build --packages-select navigation2 --symlink-install-->
source ./install/setup.bash
```

## 测试
```
ros2 launch nav2_bringup fuben_tb3_simulation_launch.py headless:=False
```

## 运行
```
ros2 launch map_generator optimize_visual.launch.py
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```


<!-- rm -rf build install log

git submodule update --init --recursive
colcon build --symlink-install
colcon build --packages-select map_generator
colcon build --packages-select navigation2 --allow-overriding navigation2

source /home/sudongxu/Documents/vln_navigation/install/setup.bash

ros2 launch map_generator optimize_visual.launch.py
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

帮我修改代码，使得navigation2和map_generator两个功能包可以通讯，导航功能包的输入为gird_map可通行图层，把导航的地图适配到map_generator这个仓库发布的grid_map的可通行图层上，在接收到了可通行图层之后，自己能够识别动态障碍物加到可通行图层上



## 快速修复：navigation2 构建报错（缺少 install 下的 package.sh）

症状：执行 `colcon build --packages-select navigation2` 时出现大量类似：
"Failed to find the following files: /home/.../install/<pkg>/share/<pkg>/package.sh"

原因：工作区中存在未清理或不完整的 `install/` 条目，且直接只构建 `navigation2` 会跳过/未构建它依赖的上游包，导致 colcon 在检查安装前缀时找不到某些 package.sh。

推荐流程（在工作区根目录执行）：

1. 清理旧构建/安装文件：
```bash
rm -rf build install log
```

2. 使用系统 ROS 环境（确保以你的 ROS 发行版替换 `humble`）：
```bash
source /opt/ros/humble/setup.bash
```

3. 安装依赖（只需一次）：
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. 按依赖关系构建 navigation2（会同时构建 navigation2 的上游包）：
```bash
# 在工作区根目录
colcon build --packages-up-to navigation2 --symlink-install
```

5. 如果需要，只构建 navigation2（若上一步已完成可直接运行）：
```bash
colcon build --packages-select navigation2 --allow-overriding navigation2
```

说明：
- `--packages-up-to navigation2` 会构建 navigation2 以及它的上游依赖，通常能避免 “缺少 package.sh” 的情况。
- 若工作区里已有与系统安装冲突的包，需要使用 `--allow-overriding` 明确覆盖系统包（但请谨慎）。
- 若仍报错，先确认 `rm -rf install` 已成功执行并重新运行第2、3步。

脚本自动化（在仓库 root 有一个辅助脚本 `scripts/clean_build_navigation2.sh`，可以直接运行）。 -->
