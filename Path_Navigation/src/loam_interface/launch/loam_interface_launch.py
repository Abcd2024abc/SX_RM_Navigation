from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    # 地图完全合格的名称为相对名称，因此可以预先培养节点的命名空间。
    # 如果发生变换（TF），目前似乎没有更好的选择
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # todo（orduno）用`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )

    start_loam_interface = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface",
        namespace=namespace,
        remappings=remappings,
        output="screen",
        parameters=[
            {
                "state_estimation_topic": "aft_mapped_to_init",
                "registered_scan_topic": "cloud_registered",
                "odom_frame": "odom",
                "base_frame": "gimbal_yaw",
                "lidar_frame": "front_mid360",
            }
        ],
    )

    ld = LaunchDescription()

    # 添加操作
    ld.add_action(declare_namespace)
    ld.add_action(start_loam_interface)

    return ld
