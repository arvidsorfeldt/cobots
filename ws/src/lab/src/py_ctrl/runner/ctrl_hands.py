import rclpy
from rclpy.node import Node
import numpy as np
from math import sqrt

from mp_msgs.msg import Hands
from std_msgs.msg import String

# landmarkList = [ wrist,
#  thumb_cmc,
#  thumb_mcp,
#  thumb_ip,
#  thumb_tip,
#  index_finger_mcp, x, y, z
#  index_finger_pip,
#  index_finger_dip,
#  index_finger_tip,
#  middle_finger_mcp,
#  middle_finger_pip,
#  middle_finger_dip,
#  middle_finger_tip,
#  ring_finger_mcp,
#  ring_finger_pip,
#  ring_finger_dip,
#  ring_finger_tip,
#  pinky_mcp,
#  pinky_pip,
#  pinky_dip,
#  pinky_tip]

class HandsSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Hands,
            '/hands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'hands_gesture', 10)

    def listener_callback(self, msg):
        try:
            x = String()
            x.data = ""

            hand = msg.hands[0] # one hand

            # fill x.data with the gesture you find.

            self.publisher_.publish(x)
        except:
            self.get_logger().info(f"I heard None")
        #self.get_logger().info(f"I heard {get_angle(get_vector(msg.hands[0].thumb_ip,msg.hands[0].thumb_tip),get_vector(msg.hands[0].pinky_mcp, msg.hands[0].index_finger_mcp))}")
        #self.get_logger().info(f"I heard {msg.hands[0].index_finger_tip.x}")



def main(args=None):
    rclpy.init(args=args)
    s = HandsSubscriber()
    rclpy.spin(s)
    s.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()