from math import sin, cos, pi
import time
import threading
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.clock import Clock


from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from rcl_interfaces.msg import ParameterEvent

from tf2_ros import TransformBroadcaster, TransformStamped

import numpy as np
from pynput import keyboard
from pynput.keyboard import Listener, Key

PI = np.pi

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        # robot state
        self.base_to_link1 = 0.0
        self.link1_to_link2 = 0.18
        self.link2_to_link3 = 1.4
        self.link3_to_link4 = 3.15

        degree = pi / 180.0
        
        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                
                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
                joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                print('send already')

                # create new robot state
                self.base_to_link1 += degree
                if self.base_to_link1 < -3.14 or self.base_to_link1 > 3.14:
                    self.base_to_link1 *= -1

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

class StatePublisher1(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher1')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.joint_state = JointState()
        self.joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
        
        self.declare_parameter('base_to_link1', 0.0)
        self.declare_parameter('link1_to_link2', 0.0)
        self.declare_parameter('link2_to_link3', 0.0)
        self.declare_parameter('link3_to_link4', 0.0)

        loop_rate = self.create_rate(30)
        self.parameter_sub = self.create_subscription(ParameterEvent, '/parameter_events', self._parameters_callbacks, 10)

        self.base_to_link1 = self.get_parameter('base_to_link1').value
        self.link1_to_link2 = self.get_parameter('link1_to_link2').value
        self.link2_to_link3 = self.get_parameter('link2_to_link3').value
        self.link3_to_link4 = self.get_parameter('link3_to_link4').value

        self.joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]
        
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
                self.joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]
                self.joint_pub.publish(self.joint_state)
                loop_rate.sleep()
        except KeyboardInterrupt:
            pass

    def _parameters_callbacks(self, messages):
        self.base_to_link1 = self.get_parameter('base_to_link1').value
        self.link1_to_link2 = self.get_parameter('link1_to_link2').value
        self.link2_to_link3 = self.get_parameter('link2_to_link3').value
        self.link3_to_link4 = self.get_parameter('link3_to_link4').value

        # self.joint_state.position = [base_to_link1, link1_to_link2, link2_to_link3, link3_to_link4]

class StatePublisher2(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher2')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)
        

        # robot state
        self.base_to_link1 = 3.0
        self.link1_to_link2 = 0.18
        self.link2_to_link3 = 1.4
        self.link3_to_link4 = 3.15

        self.declare_parameter('base_to_link1', 0.0)

        # self.parameter_sub = self.create_subscription(ParameterEvent, '/parameter_events', self._parameters_callbacks, qos_profile)
        # self.timer = self.create_timer(1, self._parameters_callbacks)

        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                
                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
                joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]
                
                # send the joint state and transform
                self.joint_pub.publish(joint_state)

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def _base_to_link1_callback(self, parameter):
        self.base_to_link1 = parameter.value
        print('11111')

    def _parameters_callbacks(self, messages):
        self.base_to_link1 = self.get_parameter('base_to_link1').value
        print('1111111111246465486312')
        # pass

class StatePublisher3(Node):
    # 尝试使用参数直接改变四个关节的角度，代码错误
    def __init__(self):
        super().__init__('state_publisher3')

        self.declare_parameter('base_to_link1', 3.0)
        self.declare_parameter('link1_to_link2', 0.0)
        self.declare_parameter('link2_to_link3', 0.0)
        self.declare_parameter('link3_to_link4', 0.0)

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.param_sub = self.create_subscription(
            ParameterEvent, '/parameter_events', self.parameter_event_callback, qos_profile)
        
        self.joint_state = JointState()
        self.update_joint_state()

        self.timer = self.create_timer(1.0 / 30, self.timer_callback)

    def update_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
        joint_state.position = [
            self.get_parameter('base_to_link1').get_parameter_value().double_value,
            self.get_parameter('link1_to_link2').get_parameter_value().double_value,
            self.get_parameter('link2_to_link3').get_parameter_value().double_value,
            self.get_parameter('link3_to_link4').get_parameter_value().double_value
        ]

        self.joint_pub.publish(joint_state)
    
    def timer_callback(self):
        self.update_joint_state()
        self.joint_pub.publish(self.joint_state)
    
    def parameter_event_callback(self, msg):
        for new_param in msg.new_parameters:
            if new_param.node == self.get_namespace():
                if new_param.name in self.joint_state.name:
                    self.update_joint_state()
                    break

class StatePublisher4(Node):
    # 尝试参数订阅,另外写个节点来发送参数
    def __init__(self):
        
        super().__init__('state_publisher4')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.subscription = self.create_subscription(
            String,
            'joint_angles',
            self.joint_angles_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # robot state
        self.base_to_link1 = 0.0
        self.link1_to_link2 = 0.18
        self.link2_to_link3 = 1.4
        self.link3_to_link4 = 3.15

    def joint_angles_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        now = self.get_clock().now()
        
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
        joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]

        # send the joint state and transform
        self.joint_pub.publish(joint_state)

        # create new robot state

        degree = pi / 180.0
        self.base_to_link1 += degree
        if self.base_to_link1 < -3.14 or self.base_to_link1 > 3.14:
            self.base_to_link1 *= -1

class StatePublisher5(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)


        # robot state
        self.base_to_link1 = 0.0
        self.link1_to_link2 = 0.18
        self.link2_to_link3 = 1.4
        self.link3_to_link4 = 3.15

        degree = pi / 180.0
        
        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                
                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
                joint_state.position = [self.base_to_link1, self.link1_to_link2, self.link2_to_link3, self.link3_to_link4]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                print('send already')

                # create new robot state
                self.base_to_link1 += degree
                if self.base_to_link1 < -3.14 or self.base_to_link1 > 3.14:
                    self.base_to_link1 *= -1

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

msg="""
接收来自键盘的信号，将信号处理后发送给仿真界面
按Ctrl+C退出
"""

class StatePublisher6(Node):
    print(msg)
    def __init__(self):
        super().__init__('StatePublisher6')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            String,
            'my_joint_states',
            self.my_joint_states_callback,
            10)
        self.curr_joint_states = None
        self.timer = self.create_timer(1/30, self.timer_callback)  # Timer to publish at 30Hz

    def my_joint_states_callback(self, msg):
        if msg.data != self.curr_joint_states:
            print('received message from keyboard!')
            self.curr_joint_states = msg.data

    def timer_callback(self):
        if self.curr_joint_states is not None:
            self.publish_realtime_joint_states()

    def publish_realtime_joint_states(self):

        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3', 'link3_to_link4']
        self.curr_joint_states
        joint_states = re.findall(r':(.*?),', self.curr_joint_states)
        joint_state.position = [float(joint_states[0]), float(joint_states[1]), float(joint_states[2]), float(joint_states[3])]
        # print(joint_states)
        self.joint_pub.publish(joint_state)

def main6(args=None):
    rclpy.init(args=args)
    node = StatePublisher6()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    node = StatePublisher5()

def main4():
    rclpy.init()
    node = StatePublisher4()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            loop_rate = node.create_rate(30)

            loop_rate.sleep()
    except KeyboardInterrupt:
        pass

def main2():
    # rclpy.init()
    # node = StatePublisher2()
    # rclpy.spin(node)
    node = StatePublisher2()

def main3(args=None):
    rclpy.init(args=args)

    node = StatePublisher3()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

