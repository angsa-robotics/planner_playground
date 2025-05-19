from os import getenv, path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get the launch directory
    log_level = LaunchConfiguration("log_level")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )
    declare_nav2_config_path = DeclareLaunchArgument(
        "nav2_config_path",
        default_value=path.join(
            get_package_share_directory("planner_playground"), "config", "nav2.yaml"
        ),
        description="Path to the nav2 config file",
    )

    lifecycle_nodes = ["planner_server", "controller_server", "bt_navigator"]
    nav2_config_path = [LaunchConfiguration("nav2_config_path")]
    load_nodes = GroupAction(
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                emulate_tty=True,
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server_amcl",
                parameters=[
                    {"topic_name": "map_amcl"},
                    {"yaml_filename": get_package_share_directory("planner_playground")
                                      + "/config/map.yaml"},
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server_amcl",
                parameters=[
                    {"bond_timeout": 0.0},
                    {
                        "autostart": True
                    },
                    {"node_names": ["map_server_amcl"]},
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=False,
                respawn_delay=2.0,
                parameters=nav2_config_path,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/map", "/map_amcl")],
                emulate_tty=True,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                respawn=False,
                respawn_delay=2.0,
                parameters=nav2_config_path,
                arguments=["--ros-args", "--log-level", log_level],
                emulate_tty=True,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=False,
                parameters=[nav2_config_path], # {"default_nav_to_pose_bt_xml": get_package_share_directory("planner_playground") + "/config/test_navigation.xml", "default_nav_through_poses_bt_xml": get_package_share_directory("planner_playground") + "/config/test_navigation.xml"}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"autostart": True},
                    {"node_names": lifecycle_nodes},
                    {"bond_timeout": 0.0},
                ],
                emulate_tty=True,
            ),
            Node(
                package="planner_playground",
                executable="simulator",
                name="simulator",
                output="screen",
                emulate_tty=True,
                parameters=[{"initial_x": 0.0, "initial_y": 0.0}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    [get_package_share_directory("planner_playground"), "/config/", "nav.rviz"],
                ],
                parameters=[{"use_sim_time": False}],
                output="screen",
                remappings=[
                    ("/goal_pose", "/goal_poses"),
                ],
                emulate_tty=True,
            ),
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_nav2_config_path)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    return ld
