#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    decision_dir = get_package_share_directory("decision_tree")
    connection_dir = get_package_share_directory("connection_layer")

    namespace = LaunchConfiguration("namespace")
    decision = LaunchConfiguration("decision")
    connection = LaunchConfiguration("connection")
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    decision_params = LaunchConfiguration("decision_params")
    connection_params = LaunchConfiguration("connection_params")

    declare_decision_config_cmd = DeclareLaunchArgument(
        "decision_params",
        default_value=os.path.join(decision_dir, "params", "DecisionParams.yaml"),
        description="决策参数文件",
    )

    declare_connection_config_cmd = DeclareLaunchArgument(
        "connection_params",
        default_value=os.path.join(connection_dir, "config", "connection_layer.yaml"),
        description="连接层参数文件",
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="命名空间",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="日志级别，可选值：debug, info, warn, error, fatal",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim time",
    )

    declare_connection_cmd = DeclareLaunchArgument(
        "connection",
        default_value="True",
        description="是否启动连接层（true/false）",
    )

    declare_decision_cmd = DeclareLaunchArgument(
        "decision",
        default_value="True",
        description="是否启动决策树（true/false）",
    )

    connection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(connection_dir, "launch", "connection_layer.launch.py")
        ),
        condition=IfCondition(PythonExpression(connection)),
        launch_arguments=[
            ("namespace", namespace),
            ("log_level", log_level),
            ("params_file", connection_params),
        ],
    )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(decision_dir, "launch", "decision_tree.launch.py")
        ),
        condition=IfCondition(PythonExpression(decision)),
        launch_arguments=[
            ("namespace", namespace),
            ("log_level", log_level),
            ("use_sim_time", use_sim_time),
            ("params_file", decision_params),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_connection_config_cmd)
    ld.add_action(declare_decision_config_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_connection_cmd)
    ld.add_action(declare_decision_cmd)

    ld.add_action(connection_launch)
    ld.add_action(decision_launch)

    return ld
