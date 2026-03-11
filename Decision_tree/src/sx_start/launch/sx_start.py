#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    param_data = {}
    sx_start_dir = get_package_share_directory("sx_start")
    config_dir = os.path.join(sx_start_dir, "config")
    launch_decision = os.path.join(get_package_share_directory("decision_tree"), "launch")
    launch_nav = os.path.join(sx_start_dir, "launch")
    launch_connection = os.path.join(get_package_share_directory("connection_layer"), "launch")

    param_file_path = os.path.join(config_dir, "sx_start_data.yaml")
    try:
        with open(param_file_path, 'r') as f:
            param_data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        print(f"未找到参数文件 {param_file_path}，使用内置默认值")

    namespace_default = str(param_data.get("namespace"))
    model_default = str(param_data.get("model"))
    decision_default = str(param_data.get("decision"))
    nav_default = str(param_data.get("nav"))
    connection_default = str(param_data.get("connection"))

    namespace = LaunchConfiguration("namespace")
    decision = LaunchConfiguration("decision")
    connection = LaunchConfiguration("connection")
    nav = LaunchConfiguration("nav")
    log_level = LaunchConfiguration("log_level")
    model = LaunchConfiguration("model")

    declare_param_file_cmd = DeclareLaunchArgument(
        "param_file",
        default_value=param_file_path,
        description="YAML参数文件，覆盖默认参数"
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=namespace_default,
        description="命名空间",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="日志级别，可选值：debug, info, warn, error, fatal",
    )
    
    declare_model_cmd = DeclareLaunchArgument(
        "model",
        default_value=model_default,
        description="是否启动仿真.",
    )
    
    declare_connection_cmd = DeclareLaunchArgument(
        "connection",
        default_value=connection_default,
        description="是否启动连接层（true/false）",
    )
    
    declare_nav_cmd = DeclareLaunchArgument(
        "nav",
        default_value=nav_default,
        description="是否启动导航（true/false）",
    )
    
    declare_decision_cmd = DeclareLaunchArgument(
        "decision",
        default_value=decision_default,
        description="是否启动决策树（true/false）",
    )

    connection_launch = TimerAction(
        period=0.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_connection, "connection_layer.launch.py")
                ),
                condition=IfCondition(PythonExpression(['"', connection, '" == "true"'])),
                launch_arguments=[
                    ("namespace", namespace),
                    ("log_level", log_level),
                ],
            )
        ]
    )

    nav_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_nav, "nav_start.py")
                ),
                condition=IfCondition(PythonExpression(['"', nav, '" == "true"'])),
                launch_arguments=[
                    ("namespace", namespace),
                    ("log_level", log_level),
                    ("model", model),
                ],
            )
        ],
    )

    decision_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_decision, "decision_tree.py")
                ),
                condition=IfCondition(PythonExpression(['"', decision, '" == "true"'])),
                launch_arguments=[
                    ("namespace", namespace),
                    ("log_level", log_level),
                ],
            )
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)
    
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_connection_cmd)
    ld.add_action(declare_nav_cmd)
    ld.add_action(declare_decision_cmd)
    
    ld.add_action(connection_launch)
    ld.add_action(nav_launch)
    ld.add_action(decision_launch)

    return ld
