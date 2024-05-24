import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from math import sin, cos, pi


class KeyboardInput(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('keyboard_input')

        self.publisher = self.create_publisher(String, 'joint_angles', 10)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        timer_period = 1/30  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_angles)

        # robot state
        self.base_to_link1 = 0.0
        self.link1_to_link2 = 0.18
        self.link2_to_link3 = 1.4
        self.link3_to_link4 = 3.15

    def publish_joint_angles(self):
        msg = String()

        degree = pi / 180.0
        self.base_to_link1 += degree
        if self.base_to_link1 < -3.14 or self.base_to_link1 > 3.14:
            self.base_to_link1 *= -1

        msg.data = f'{self.base_to_link1} {self.link1_to_link2} {self.link2_to_link3} {self.link3_to_link4}'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main():
    node = KeyboardInput()
    rclpy.spin(node)
    KeyboardInput.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()