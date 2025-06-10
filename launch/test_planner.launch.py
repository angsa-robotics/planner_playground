from os import getenv, path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

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
        description="Path to the nav2 config file",
    )

    declare_ros2_control_config = DeclareLaunchArgument(
        "ros2_control_config",
        description="Path to the ros2_control config file",
    )


    lifecycle_nodes = ["planner_server", "bt_navigator"]
    nav2_config_path = [LaunchConfiguration("nav2_config_path")]
    ros2_control_config_path = [LaunchConfiguration("ros2_control_config")]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_config_path,
            root_key="",
            param_rewrites={"default_nav_through_poses_bt_xml": path.join(
                get_package_share_directory("planner_playground"), "config", "test_navigation.xml"
                )},
            convert_types=True,
        ),
        allow_substs=True,
    )
    nav2 = GroupAction(
        actions=[
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
                parameters=[{"autostart": True, "node_names": ["map_server_amcl"], "bond_timeout": 0.0}],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[*nav2_config_path, path.join(
                get_package_share_directory("planner_playground"), "config", "nav2_overrides.yaml"
                )],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("/map", "/map_amcl")],
                emulate_tty=True,
            ),
            # Node(
            #     package="nav2_controller",
            #     executable="controller_server",
            #     name="controller_server",
            #     output="screen",
            #     parameters=[*nav2_config_path, path.join(
            #     get_package_share_directory("planner_playground"), "config", "nav2_overrides.yaml"
            #     )],
            #     arguments=["--ros-args", "--log-level", log_level],
            #     emulate_tty=True,
            # ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"autostart": True, "node_names": lifecycle_nodes, "bond_timeout": 0.0},
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
            ExecuteProcess(
                cmd=["ros2", "bag", "play", path.join(
                get_package_share_directory("planner_playground"), "config", "path.mcap")],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )

    robot_description_content = Command(
        [
            "xacro ",
            path.join(
                get_package_share_directory("planner_playground"), "config", "description.xacro"
                )
        ]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
    )

    ros2_control = GroupAction(
        actions=[
            Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    parameters=[
                        {
                            "publish_frequency": 10.0,
                            "robot_description": robot_description_content,
                        }
                    ],
                    output="screen",
                    emulate_tty=True,
                ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[*ros2_control_config_path, path.join(
                get_package_share_directory("planner_playground"), "config", "ros2_control_overrides.yaml"
                )],
                output="screen",
                remappings=[
                    ("~/robot_description", "/robot_description"),
                    ("~/cmd_vel", "/cmd_vel"),
                    ("~/odom", "/odometry/encoders"),
                ],
                emulate_tty=True,
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "diff_drive_controller",
                ],
                emulate_tty=True,
            ),
        ],
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_nav2_config_path)
    # ld.add_action(declare_ros2_control_config)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(nav2)
    ld.add_action(static_tf)
    # ld.add_action(ros2_control)
    return ld
