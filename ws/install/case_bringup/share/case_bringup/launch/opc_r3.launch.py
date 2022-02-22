import sys
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

def generate_launch_description():

    opcua_parameters = {
        "server_address": "opc.tcp://192.168.100.30:4840/",
        "node_ids": ["ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_1",
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_2", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_3", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_4", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_from_plc_5", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_1",
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_2", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_3", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_4", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_from_plc_5",
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_1",
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_2", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_3", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_4", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.bool_to_plc_5", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_1",
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_2", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_3", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_4", 
                     "ns=4;s=|var|CODESYS CONTROL FOR Raspberry Pi MC SL.Application.IO.int_to_plc_5"
                     ]
    }

    opc_node = Node(
        package="opcua_ros2_bridge",
        executable="opcua_ros2_bridge",
        namespace="",
        output="screen",
        arguments=["-d"],
        parameters=[opcua_parameters],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        emulate_tty=True,
    )

    nodes_to_start = [
        opc_node
    ]
    
    return LaunchDescription(nodes_to_start)


if __name__ == "__main__":
    ls = LaunchService(argv=sys.argv[1:])
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())
