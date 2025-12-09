# README.md

一. 项目作用
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

