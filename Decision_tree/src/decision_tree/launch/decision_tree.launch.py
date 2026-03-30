import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LoadComposableNodes, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

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
        default_value="true",
        description="Use sim time"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "params", "DecisionParams.yaml"),
        description="Params file path"
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                name="decision_tree",
                package="decision_tree",
                executable="decision_tree_node",
                output="screen",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file)
    # ld.add_action(container)
    # ld.add_action(load_component)
    ld.add_action(bringup_group)

    return ld