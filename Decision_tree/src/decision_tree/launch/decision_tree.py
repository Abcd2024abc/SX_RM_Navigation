from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction

def generate_launch_description():
    # 接收参数
    namespace = LaunchConfiguration("namespace")

    # 声明参数
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="red_standard_robot1",
        description="命名空间."
    )

    # decision_tree
    decision_tree = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        SetRemap("/tf", "tf"),
        SetRemap("/tf_static", "tf_static"),
        Node(
            package='decision_tree',
            executable='decision_tree',
            name='decision_tree',
            output='screen',
            parameters=[{
                'namespace': LaunchConfiguration('namespace'),
            }]
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(decision_tree)

    return ld