# TB3-Gazebo-Nav2-explore-lite
TB3 + Gazebo + Nav2 + explore_lite 仿真与自主探索

## 安装配置

### 仿真环境前置要求：按照教程安装vln_gazebo_simulator
```bash
参考：https://github.com/Tipriest/vln_gazebo_simulator
参考放置目录: ~/Documents/vln_gazebo_simulator #或你的工作空间路径
```
### 安装Nav2和TB3
``` bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-turtlebot3-gazebo
```
### 下载explore源码
``` bash
cd ~/Documents/vln_gazebo_simulator/src #或你的工作空间路径
git clone https://github.com/jiangn19/TB3-Gazebo-Nav2-explore-lite
git submodule update --init --recursive
```
### 编译
``` bash
cd ..
source /opt/ros/humble/setup.bash 
colcon build --symlink-install
# colcon build --symlink-install --packages-select explore_lite
```
### 配置路径依赖
``` bash
chmod +x ~/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/setup_explore.bash #或你的工作空间路径
```
在bashrc中添加下面一行：
``` bash
source ~/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/setup_explore.bash  #或你的工作空间路径
```
``` bash
source ~/.bashrc
turtle3init
```
### 测试
``` bash
# tb3功能性测试
# ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
# ros2 launch tb3_simulation tb3_simulation_house_launch.py slam:=True
```
## 参数
Nav2与explorelite默认参数位于config文件夹中
启动节点时，使用'params_file:='指定加载的参数位置

## 运行探索
### 运行仿真场景
``` bash
turtle3init
# 启动00829场景 tb3 + gazebo场景仿真
ros2 launch turtlebot3_gazebo turtlebot3_00829.launch.py
```
### 启动slam_toolbox建图
``` bash
turtle3init
# 启动slam-toolbox在线建图（使用仿真时间）
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
### 启动nav2导航
<!-- 需要按照自己的路径更改一下params_file对应的 nav2_params.yaml 的路径
初始路径在ros humble对应的安装路径下的nav2_bringup包里面
默认为：/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
不建议直接在系统文件下更改，可以直接复制一个副本，放在任意你喜欢的路径下s
为了好找，建议放在工作路径~/Documents/vln_gazebo_simulator/nav2_params.yaml这里 -->
``` bash
turtle3init
```
``` bash
# default
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params.yaml # 或你自己的param路径
```

### 启动explore_lite自主探索
```bash
turtle3init
```
``` bash
# 使用自定义params文件启动
ros2 launch explore_lite explore.launch.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/params_costmap.yaml
```

### 地图保存与加载
``` bash
# <!-- ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: '/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/saved_map/'}" -->

ros2 run nav2_map_server map_saver_cli -f /home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/map/map1
``` 

``` bash

# <!-- ros2 launch nav2_bringup navigation_launch.py \
#   use_sim_time:=True \
#   autostart:=False \
#   map:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/saved_map/map1216.yaml \
#   params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params_pnc.yaml -->

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  autostart:=False \
  map:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/map/map1216.yaml \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params_pnc_dwb.yaml

# <!-- ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True autostart:=False map:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/saved_map/map1.yaml -->
```

### 启动纯导航
```bash
turtle3init
```

``` bash
ros2 launch nav2_bringup rviz_launch.py
```

``` bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params_pnc_dwb.yaml
```


### 启动纯导航(修改了规划逻辑)
```bash
turtle3init
```

``` bash
ros2 launch nav2_bringup rviz_launch.py
```

``` bash
ros2 launch nav2_bringup bringup_launch_without_smooth.py \
  use_sim_time:=True \
  autostart:=False \
  map:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/saved_map/map1216.yaml \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params_pnc_dwb.yaml
```

``` bash
ros2 launch nav2_bringup navigation_launch_without_smooth.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2_params_pnc_dwb.yaml
```