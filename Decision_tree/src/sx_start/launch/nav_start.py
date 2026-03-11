import os
import yaml
from launch.actions import TimerAction

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    launch_nav = os.path.join(get_package_share_directory("nav_bringup"), "launch")
    launch_gazebo = os.path.join(get_package_share_directory("rmu_gazebo_simulator"), "launch")
    config_dir = os.path.join(get_package_share_directory("sx_start"), "config")
    param_file = os.path.join(config_dir, "nav_start_data.yaml")

    # 读取参数文件
    with open(param_file, 'r') as f:
        param_data = yaml.safe_load(f)

    # 使用参数文件中的值作为默认值
    world_default = str(param_data.get("world"))
    namespace_default = str(param_data.get("namespace"))
    slam_default = str(param_data.get("slam"))
    use_robot_state_pub_default = str(param_data.get("use_robot_state_pub"))
    use_rviz_default = str(param_data.get("use_rviz"))
    model_default = str(param_data.get("model"))

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    # 声明参数文件
    declare_param_file_cmd = DeclareLaunchArgument(
        "param_file",
        default_value=param_file,
        description="YAML参数文件，覆盖默认参数"
    )
    declare_model_cmd = DeclareLaunchArgument(
        "model",
        default_value=model_default,
        description="是否启动仿真."
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=world_default,
        description="地图文件选择."
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value=namespace_default,
        description="机器人命名空间."
    )
    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value=slam_default,
        description="是否使用SLAM."
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value=use_robot_state_pub_default,
        description="是否使用静态TF."
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value=use_rviz_default,
        description="是否使用RViz."
    )

    # simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_nav, "rm_navigation_simulation_launch.py")
        ),
        condition=IfCondition(model),
        launch_arguments=[
            ("world", world),
            ("namespace", namespace),
            ("slam", slam),
            ("use_rviz", use_rviz),
        ],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_gazebo, "bringup_sim.launch.py")
        ),
        condition=IfCondition(model),
    )

    # reality
    reality_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_nav, "rm_navigation_reality_launch.py")
        ),
        condition=IfCondition(PythonExpression(['not ', model])),
        launch_arguments=[
            ("world", world),
            ("namespace", namespace),
            ("slam", slam),
            ("use_robot_state_pub", use_robot_state_pub),
            ("use_rviz", use_rviz),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(gazebo_launch)
    ld.add_action(simulation_launch)
    ld.add_action(reality_launch)
    return ld
