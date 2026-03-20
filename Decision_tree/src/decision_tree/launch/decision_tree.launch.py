import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction

def generate_launch_description():
    pkg_dir = get_package_share_directory("decision_tree")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    # 声明参数
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use sim time"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "params", "DecisionParams.yaml"),
        description="Params file path"
    )

    # 使用组件容器
    container = Node(
        name='decision_tree_container',
        package='rclcpp_components',
        namespace=namespace,
        executable='component_container',
        output='screen',
        parameters=[params_file],
        arguments=["--fos-args", "--log-level", "info"]
    )
    
    # 加载决策树组件
    load_component = LoadComposableNodes(
        target_container='decision_tree_container',
        composable_node_descriptions=[
            ComposableNode(
                package='decision_tree',
                plugin='decision_tree::DecisionTree',
                name='decision_tree',
                namespace=namespace,
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file)
    ld.add_action(container)
    ld.add_action(load_component)

    return ld