# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =========================
# 导入区：按功能分组说明
# - ament_index: 查找ROS包share目录
# - launch核心/动作/条件/替换：描述Launch图、定义行为、按条件启用、参数/字符串替换
# - launch_ros：ROS2节点、命名空间、参数文件等
# - nav2_common：常用替换工具（字符串与YAML参数重写）
# =========================
import os

from ament_index_python.packages import get_package_share_directory  # 获取包的share路径

from launch import LaunchDescription  # 顶层Launch描述对象
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)  # 参数声明/分组/引入子launch/环境变量
from launch.conditions import IfCondition  # 条件启用动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指向.py类型的Launch源
from launch.substitutions import LaunchConfiguration, PythonExpression  # 惰性配置与表达式求值
from launch_ros.actions import Node  # 启动ROS2节点
from launch_ros.actions import PushRosNamespace  # 推送命名空间
from launch_ros.descriptions import ParameterFile  # 参数文件包装（支持替换）
from nav2_common.launch import ReplaceString, RewrittenYaml  # 字符串替换与YAML参数重写


def generate_launch_description():
    """
    每段功能概要：
    - 目录定位：bringup_dir / launch_dir
    - 参数配置：通过 LaunchConfiguration 承接命令行或默认值
    - TF重映射：保证命名空间场景下的tf正确前缀
    - 参数重写：将 use_sim_time 与 yaml_filename 注入参数树；按需替换 <robot_namespace>
    - 参数声明：提供默认值与描述，支持命令行覆盖
    - 主组装：可选命名空间、可选组合容器、SLAM或Localization子系统、Navigation子系统
    - 顶层装配：环境变量、参数声明、Bringup主组动作
    """
    # =========================
    # 目录定位：用于拼接其他被引入的 launch 路径
    # =========================
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # =========================
    # Launch 参数获取区（惰性）：这些值在运行时由命令行或默认值注入
    # - namespace/use_namespace：多机器人或命名空间隔离
    # - slam：Slam vs Localization 选择
    # - map_yaml_file：Localization所需静态地图
    # - use_sim_time：仿真时间绑定
    # - params_file：全局参数文件入口
    # - autostart：生命周期自动上电
    # - use_composition：组件化容器（减少进程）
    # - use_respawn：崩溃自动重启
    # - log_level：日志级别
    # =========================
    namespace = LaunchConfiguration('namespace')        # 机器人命名空间
    use_namespace = LaunchConfiguration('use_namespace')# 是否启用命名空间（布尔）
    slam = LaunchConfiguration('slam')                  # 是否启用 SLAM（True/False）
    map_yaml_file = LaunchConfiguration('map')          # 地图 YAML 文件路径（用于定位模式）
    use_sim_time = LaunchConfiguration('use_sim_time')  # 是否使用仿真时间
    params_file = LaunchConfiguration('params_file')    # 全局参数文件路径
    autostart = LaunchConfiguration('autostart')        # 生命周期自动启动
    use_composition = LaunchConfiguration('use_composition')  # 组合方式加载
    use_respawn = LaunchConfiguration('use_respawn')    # 节点崩溃是否自动重启
    log_level = LaunchConfiguration('log_level')        # 全局日志等级

    # =========================
    # TF 重映射：
    # - tf/tf_static 使用相对命名，便于在 namespace 下自动加前缀
    # - 避免绝对话题名打破命名空间隔离
    # =========================
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # =========================
    # 参数重写/替换区：
    # - param_substitutions：向参数树注入 use_sim_time / yaml_filename
    # - ReplaceString：仅在 use_namespace=True 时把 <robot_namespace> 实际替换为 /<namespace>
    #   否则保持原样（condition会跳过替换）
    # - RewrittenYaml：对原参数文件做按键重写，并以 namespace 作为根键实现多机隔离
    # - ParameterFile(allow_substs=True)：允许在YAML中继续引用Launch替换（例如 LaunchConfiguration）
    # =========================
    param_substitutions = {
        'use_sim_time': use_sim_time,     # 统一仿真时间
        'yaml_filename': map_yaml_file}   # 静态地图路径

    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace))  # 当未启用命名空间时不做替换

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,                 # 以 namespace 为根：/namespace/...
            param_rewrites=param_substitutions, # 注入/覆盖键
            convert_types=True),                # 自动把 'True'/'False' 等转为布尔/数值类型
        allow_substs=True)                      # 允许二次替换（如 LaunchConfiguration 嵌入）

    # =========================
    # 环境变量：
    # - 行缓冲：减少日志延迟
    # =========================
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # =========================
    # 参数声明区：
    # - 提供默认值与说明，允许命令行覆盖
    # - 类型与校验交由下游节点或launch表达式处理
    # =========================
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')  # 空字符表示不启用命名空间

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')  # True: 以组件方式加载，减少进程

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # =========================
    # Bringup 主组动作：
    # - PushRosNamespace：条件性启用命名空间（仅当 use_namespace=True）
    # - 组件容器：use_composition=True 时启动 component_container_isolated
    #   与 component_container 区别：每组件独立回调组隔离
    # - SLAM：slam=True 引入 slam_launch.py（map_saver/slam_toolbox 等）
    # - Localization：否则引入 localization_launch.py（map_server + amcl）
    # - Navigation：无论上述选择如何，都引入 navigation_launch.py（规划/控制/BT等）
    # 引入顺序有助于依赖就绪，但ROS2生命周期会确保正确状态流转。
    # =========================
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),  # 之后的所有节点/子launch均在该命名空间下

        # 在使用组合模式时，先启动一个组件容器（nav2_container）
        # 后续 navigation_launch / localization_launch 会根据 use_composition 决定把组件加载到这个容器
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',  # 组件容器（隔离版）
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        # 如果 slam=True，则引入 slam_launch.py，提供 map_saver、slam_toolbox 等
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),  # 仅当 slam=True
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        # 如果未启用 slam，则引入 localization_launch.py，使用静态地图做定位
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),  # 当未启用SLAM
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,    # 传递静态地图
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        # 无论是否 SLAM/Localization，都需要 navigation 栈（规划、控制、BT、waypoint follower 等）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch_without_smooth.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # =========================
    # 顶层装配：
    # - 注册环境变量
    # - 注册所有参数声明（便于命令行覆盖）
    # - 注册主组动作
    # 返回 LaunchDescription
    # =========================
    ld = LaunchDescription()

    # 环境变量设置（日志缓冲）
    ld.add_action(stdout_linebuf_envvar)

    # 将所有参数声明添加到 LaunchDescription，使其可从命令行覆盖
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 栈的主体：包含（可选的）namespace、容器、slam/localization、navigation 等
    ld.add_action(bringup_cmd_group)

    return ld
