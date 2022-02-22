import rclpy
import json
import time
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal')

    publisher = node.create_publisher(String, '/set_state', 10)
    msg = String()
    pos = {
        "trigger_goal_pos2": True
    }

    msg.data = json.dumps(pos)
    time.sleep(0.1)
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()