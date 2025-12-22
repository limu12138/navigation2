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

import os

from ament_index_python.packages import get_package_share_directory  # 获取包的 share 路径

from launch import LaunchDescription                           # LaunchDescription：Launch 图的根对象
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable  # 参数声明/分组/环境变量
from launch.conditions import IfCondition                      # 条件控制（运行时求值）
from launch.substitutions import LaunchConfiguration, PythonExpression  # 惰性配置与表达式（运行期求值）
from launch_ros.actions import LoadComposableNodes             # 组合节点加载动作
from launch_ros.actions import Node                            # 普通进程节点
from launch_ros.descriptions import ComposableNode, ParameterFile  # 组合节点描述/参数文件包装
from nav2_common.launch import RewrittenYaml                   # YAML 参数按键重写工具

def generate_launch_description():
    """
    生成 Navigation 子栈的 LaunchDescription，负责装配控制/规划/行为树/路径点/速度平滑等组件。
    主要职责：
    - 解析 Launch 参数（命名空间、仿真时间、参数文件、是否组合、容器名、重启策略、日志级别）
    - 处理 TF 话题重映射（使用相对名以适应命名空间）
    - 通过 RewrittenYaml+ParameterFile 将 use_sim_time/autostart 注入参数树（支持类型转换）
    - 非组合模式：以独立进程启动所有导航相关节点，并交给生命周期管理器
    - 组合模式：将各节点以 ComposableNode 方式装载到指定容器
    - 设置必要的环境变量（行缓冲），并返回顶层 LaunchDescription
    注意：仅添加注释，不修改任何默认行为或参数。
    """
    # 获取 nav2_bringup 的 share 路径（用于定位默认参数文件等）
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch 参数（惰性求值，运行时从命令行或默认值注入）
    namespace = LaunchConfiguration('namespace')        # 机器人命名空间（字符串，可能为空）
    use_sim_time = LaunchConfiguration('use_sim_time')  # 使用仿真时间（布尔）
    autostart = LaunchConfiguration('autostart')        # 生命周期管理是否自动上电（布尔）
    params_file = LaunchConfiguration('params_file')    # 全局参数文件路径（YAML）
    use_composition = LaunchConfiguration('use_composition')  # 是否以组件方式加载（布尔）
    container_name = LaunchConfiguration('container_name')    # 组件容器名称（字符串）
    container_name_full = (namespace, '/', container_name)    # 完整容器名（命名空间 + 名称），作为 LoadComposableNodes 目标
    use_respawn = LaunchConfiguration('use_respawn')    # 是否在进程节点崩溃后重启（布尔，仅非组合模式生效）
    log_level = LaunchConfiguration('log_level')        # 日志等级（info/debug/warn/error/fatal）

    # 生命周期管理器将管理这些节点（进入/退出各生命周期状态）
    lifecycle_nodes = ['controller_server',
                       # 移除 smoother_server
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # 将绝对话题重映射为相对话题名，便于命名空间前缀自动生效
    # TF 类话题必须使用这种方式来保持与命名空间兼容
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 参数替换：向参数树统一注入 use_sim_time 与 autostart
    param_substitutions = {
        'use_sim_time': use_sim_time,  # 统一所有节点的时间源（仿真/实机）
        'autostart': autostart         # 通知生命周期管理器自动上电
    }

    # 基于原始参数文件构造“可替换”的参数视图：
    # - root_key=namespace：把参数树挂接在命名空间根键下（与话题命名空间相辅相成）
    # - param_rewrites：注入/覆盖上面的替换键
    # - convert_types=True：把字符串 'True'/'False'/'1' 等转换为布尔/数值类型
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)  # 允许 YAML 中继续引用 Launch 替换（如 LaunchConfiguration）

    # 设置行缓冲，减少日志刷出延迟
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # 下面声明一系列 Launch 参数，提供默认值和帮助文本，允许命令行覆盖
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')  # 空字符串表示不使用命名空间

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')  # 使用仿真时钟

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')  # 默认参数文件

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')  # 生命周期自动上电

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')  # 组合模式切换

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')  # 组合容器名

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')  # 非组合进程崩溃重启

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')  # 可选：debug/info/warn/error/fatal

    # 非组合模式：以进程节点方式启动所有组件，并附加生命周期管理器
    # 条件 IfCondition(PythonExpression(['not ', use_composition])) 保证与组合模式互斥
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # 控制器节点：跟踪目标轨迹，输出速度；cmd_vel 重映射到 cmd_vel_nav 以避免与速度平滑器冲突
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,             # 仅非组合模式有效
                respawn_delay=2.0,               # 重启延时
                parameters=[configured_params],  # 注入统一参数
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # 规划器：生成从当前位姿到目标的全局路径
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 行为服务器：封装备份/旋转/等待等原子行为
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 行为树导航器：调度规划/控制/恢复等模块，执行BT
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 路径点跟随：依次跟踪多个 waypoint
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 速度平滑器：对 cmd_vel_nav 进行平滑并输出到 cmd_vel
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            # 生命周期管理器：统一管理上述节点的生命周期状态机
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},  # 生命周期管理器自身也要匹配时间源
                            {'autostart': autostart},       # 自动上电
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    # 组合模式：以 ComposableNode 方式将节点装载至 container_name_full 容器
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),           # use_composition=True 时启用
        target_container=container_name_full,             # 目标容器（通常由 bringup_launch 创建）
        composable_node_descriptions=[
            # 控制器组件
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # 规划器组件
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            # 行为组件
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            # BT 导航组件
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            # 路径点跟随组件
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            # 速度平滑器组件
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            # 生命周期管理器组件（注意：组件方式下为插件形式）
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # 创建顶层 LaunchDescription，并依次添加环境变量、参数声明与具体动作
    ld = LaunchDescription()

    # 环境变量设置（行缓冲）
    ld.add_action(stdout_linebuf_envvar)

    # 声明所有 Launch 参数（允许命令行覆盖默认）
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 根据 use_composition 条件，仅有一个分支生效（互斥）
    ld.add_action(load_nodes)             # 非组合模式（进程节点）
    ld.add_action(load_composable_nodes)  # 组合模式（组件节点）

    return ld
