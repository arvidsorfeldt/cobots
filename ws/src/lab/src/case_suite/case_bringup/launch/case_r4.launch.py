import os
import sys
import json
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    scenario = "scenario_2"
    tf_scene_dir = FindPackageShare("case_scenes").find("case_scenes")
    bringup_dir = FindPackageShare("case_bringup").find("case_bringup")
    ur_setup_dir = FindPackageShare("ur_setup").find("ur_setup")
    ursg_setup_dir = FindPackageShare("ur_script_generator").find("ur_script_generator")

    robot_parameters_path = os.path.join(
        ur_setup_dir, "robots", "case_r4", "general.json"
    )

    parameters = {
        "templates_path": os.path.join(ursg_setup_dir),
        "scenario_path": os.path.join(tf_scene_dir, "scenarios", scenario), 
        "meshes_path": "/ros/ia_ros_meshes"
    }

    opcua_parameters = {
        "server_address": "opc.tcp://192.168.100.40:4840/",
        "node_ids": ["ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_1","ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_2", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_3", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_4", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_5", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_1","ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_2", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_3", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_4", "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_5"]
    }

    with open(robot_parameters_path) as jsonfile:
        robot_parameters = json.load(jsonfile)

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value=robot_parameters["ur_type"],
            description="Type/series of used UR robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",  # robot_parameters["prefix"],
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            robot_parameters["name"],
            "joint_limits.yaml",
        ]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            robot_parameters["name"],
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            robot_parameters["name"],
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_setup"),
            "robots",
            robot_parameters["name"],
            "visual_parameters.yaml",
        ]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare(description_package), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_2.rviz")

    driver_parameters = {
        "ur_address": robot_parameters["ip_address"],
        "tf_prefix": robot_parameters["prefix"],
    }

    ur_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="screen",
        parameters=[robot_description],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    ur_script_generator_node = Node(
        package="ur_script_generator",
        executable="ur_script_generator",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    ur_script_driver_node = Node(
        package="ur_script_driver",
        executable="ur_script_driver",
        namespace="",
        output="screen",
        parameters=[driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    ur_script_controller_node = Node(
        package="ur_script_controller",
        executable="ur_script_controller",
        namespace="",
        output="screen",
        parameters=[driver_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_lookup_node = Node(
        package="tf_lookup",
        executable="tf_lookup",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_broadcast_node = Node(
        package="tf_broadcast",
        executable="tf_broadcast",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    tf_sms_node = Node(
        package="tf_sms",
        executable="tf_sms",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_static_node = Node(
        package="viz_static",
        executable="viz_static",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_interactive_node = Node(
        package="viz_interactive",
        executable="viz_interactive",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    viz_dynamic_node = Node(
        package="viz_dynamic",
        executable="viz_dynamic",
        namespace="",
        output="screen",
        parameters=[parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    control_gui_node = Node(
        package="gui_tools",
        executable="control_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    query_gui_node = Node(
        package="gui_tools",
        executable="query_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    marker_gui_node = Node(
        package="gui_tools",
        executable="marker_gui",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    opc_node = Node(
        package="opcua_ros2_bridge",
        executable="opcua_ros2_bridge",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[opcua_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    aruco_node = Node(
        package="aruco_handler",
        executable="new_handler",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[opcua_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    aruco_locker = Node(
        package="aruco_handler",
        executable="aruco_locker",
        namespace="",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[opcua_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [
        ur_robot_state_publisher_node,
        ur_script_generator_node,
        ur_script_driver_node,
        ur_script_controller_node,
        tf_lookup_node,
        tf_broadcast_node,
        tf_sms_node,
        rviz_node,
        viz_static_node,
        viz_interactive_node,
        viz_dynamic_node,
        control_gui_node,
        query_gui_node,
        marker_gui_node,
        # opc_node, # lauch the opc node from another launch file since the plc dies after a while
        aruco_node,
        aruco_locker
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
