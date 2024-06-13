import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import time
import re

class SocketClientNode(Node):

    def __init__(self):
        super().__init__('socket_client_node')

        connected = False
        retries = 0
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not connected and retries < 5:
            try:
                retries += 1
                self.client_socket.connect(('192.168.1.108', 9090))
                connected = True
            except Exception as e:
                time.sleep(2)
                self.get_logger().info('Could not connect to server: %s' % str(e))
        
        if not connected:
            self.get_logger().info('Failed to connect after multiple attempts. Exiting node.')
            rclpy.shutdown()
        
        self.subscription = self.create_subscription(
            String,
            'my_joint_states',
            self.my_joint_states_callback,
            10)
        
        self.curr_joint_states = None
        self.last_joint_states = None
    
    def to_lower_computer(self, maplist=[1,1,1,1]):
        def process_diff(curr_value, last_value, index):
            diff = curr_value - last_value
            diff *= maplist[index]
            if abs(diff) > 0.001:
                sign = 1 if abs(diff) == diff else 0
                print(diff)
                return f'Motor{index}={round(abs(diff) * 180 / 3.14)},{sign}'
            return None

        if self.last_joint_states is not None:
            curr_joint_states_value = list(map(float, re.findall(r':(.*?),', self.curr_joint_states)))
            last_joint_states_value = list(map(float, re.findall(r':(.*?),', self.last_joint_states)))

            processed_msg_list = [process_diff(curr_joint_states_value[i], last_joint_states_value[i], i) for i in range(4) ]
            processed_msg = '\n'.join([item for item in processed_msg_list if item])
            
            return processed_msg
        else:
            return ''
        

    def my_joint_states_callback(self, msg):
        if msg.data != self.curr_joint_states:
            print('received message from keyboard!')
            self.last_joint_states = self.curr_joint_states
            self.curr_joint_states = msg.data
            processed_msg = self.to_lower_computer(maplist=[1,100,1,1])
            self.client_socket.sendall(processed_msg.encode())

    def send_message(self, msg):
        self.get_logger().info('Sending message: %s' % msg)
        self.client_socket.sendall(msg.encode())

def main(args=None):
    rclpy.init(args=args)
    node = SocketClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()