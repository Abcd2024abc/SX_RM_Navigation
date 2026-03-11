#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    生成启动描述
    """
    # ==================== 获取包路径 ====================
    # 获取 connection_layer 包的共享目录路径
    pkg_connection_layer_dir = get_package_share_directory("connection_layer")

    namespace = LaunchConfiguration("namespace")
    # params_file = LaunchConfiguration("params_file")
    log_level = LaunchConfiguration("log_level", default="info")

    # configured_params = ParameterFile(
    #     RewrittenYaml(
    #         source_file=params_file,       # 参数文件
    #         root_key=namespace,            # 根据命名空间重写根键
    #         param_rewrites={},             # 参数重写规则（空字典表示无重写）
    #         convert_types=True,            # 自动转换参数类型
    #     ),
    #     allow_substs=True,                 # 允许在参数文件中使用替换变量
    # )

    # 设置ROS2日志输出为缓冲模式
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )
    
    # 彩色日志输出
    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
    
    # 命名空间参数
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="", 
        description="命名空间",
    )

    # 参数文件路径参数：指定配置文件的路径
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg_connection_layer_dir,
            "config", 
            "connection_layer.yaml",
        ),
        description="文件路径，包含所有节点的ROS2参数配置",
    )

    # 日志级别参数：控制节点的日志输出详细程度
    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", 
        default_value="info",                  # 默认信息级别
        description="log level"                # 可选值：debug, info, warn, error, fatal
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            
            # 启动connection_layer节点
            Node(
                package="connection_layer",
                executable="connection_layer_node",
                name="connection_layer",  
                output="screen",  
                respawn=True,
                respawn_delay=2.0,
                # parameters=[configured_params], 
                arguments=["--ros-args", "--log-level", log_level], 
            ),
        ]
    )
    
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    ld.add_action(bringup_cmd_group)

    return ld