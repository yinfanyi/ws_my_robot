from rclpy.node import Node
from std_msgs.msg import String
from math import sin, cos, pi, sqrt, atan2

import sys
import threading


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


from rclpy.node import Node
from std_msgs.msg import String
from math import sin, cos, pi

import sys
import threading

import geometry_msgs.msg
import rclpy


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg="""
u i o p increase joint
m j k l decrease joint
笛卡尔空间单位位移5mm
小键盘数字键控制笛卡尔空间前后左右
  ↑      8
← ↓ →  4 5 6
7 9 控制末端位姿空间上下
ctrl-c to break
"""
jointBindings = {
    'u': 'joint_1_increase',
    'i': 'joint_2_increase',
    'o': 'joint_3_increase',
    'p': 'joint_4_increase',
    'h': 'joint_1_decrease',
    'j': 'joint_2_decrease',
    'k': 'joint_3_decrease',
    'l': 'joint_4_decrease',
    'U': 'joint_1_increase',
    'I': 'joint_2_increase',
    'O': 'joint_3_increase',
    'P': 'joint_4_increase',
    'H': 'joint_1_decrease',
    'J': 'joint_2_decrease',
    'K': 'joint_3_decrease',
    'L': 'joint_4_decrease',
    'r': 'reset',
    'R': 'reset',
}

moveBindings={
    '8': 'move_forward',
    '5': 'move_backward',
    '4': 'move_left',
    '6': 'move_right',
    '7'     : 'move_up',
    '9'     : 'move_down',
}

current_mode = 'joint_space_mode'
joint1_state = 0.
joint2_state = 0.
joint3_state = 0.
joint4_state = 0.

x=0
y=0
z=0
l1 = 0.38
l2 = 0.28
unit_displacement = 0.005


def forward_kinematic(theta1, theta2, theta4, d3):
    x = l2*cos(theta1+theta2)+l1*cos(theta1)
    y = l2*sin(theta1+theta2)+l1*sin(theta1)
    z = d3
    phi = theta1 + theta2 + theta4
    return x,y,z,phi

def inverse_kinematic(x, y, z, phi):
    try:
        c2 = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
        s2 = sqrt(1-c2**2)

        theta2 = atan2(s2,c2)

        k1 = l1 + l2*c2
        k2 = l2*s2
        theta1 = atan2(y, x) - atan2(k2, k1)
        theta4 = phi - theta1 - theta2
        d3 = z
        return theta1, theta2, theta4, d3
    except:
        return -99,-99,-99,-99

def is_valid(theta1, theta2, theta4, d3):
    if theta1 < 3 and theta1 > -3 and theta2 < 1.4 and theta2 > -1.4 and theta4 > -3.15 and theta4 < 3.15 and d3 > -0.13 and d3 < 0.18:
        return True
    else:
        return False

def update_cartesian_state(message:str):
    global joint1_state, joint2_state, joint3_state, joint4_state, x, y, z, phi
    
    x,y,z,phi = forward_kinematic(theta1 = joint1_state, theta2 = joint3_state, theta4 = joint4_state, d3 = joint2_state)

    if message.startswith('move_') and message.endswith('forward'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x+0.05,y,z,phi)
        if is_valid(theta1,theta2,theta4,d3):
            x += 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3            
    elif message.startswith('move_') and message.endswith('backward'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x-0.05,y,z,phi)
        if is_valid(theta1,theta2,theta4,d3):
            x -= 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3
    elif message.startswith('move_') and message.endswith('left'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x,y+0.05,z,phi)
        if is_valid(theta1,theta2,theta4,d3):
            y += 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3
    elif message.startswith('move_') and message.endswith('right'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x,y-0.05,z,phi)
        if is_valid(theta1,theta2,theta4,d3):
            y -= 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3
    elif message.startswith('move_') and message.endswith('up'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x,y,z+0.05,phi)
        if is_valid(theta1,theta2,theta4,d3):
            z += 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3
    elif message.startswith('move_') and message.endswith('down'):
        theta1,theta2,theta4,d3 = inverse_kinematic(x,y,z-0.05,phi)
        if is_valid(theta1,theta2,theta4,d3):
            z -= 0.05
            joint1_state = theta1 
            joint3_state = theta2
            joint4_state = theta4
            joint2_state = d3

    print(f'Updated joint states: \njoint1_state:{joint1_state:.3f}, \njoint2_state:{joint2_state:.3f}, \njoint3_state:{joint3_state:.3f}, \njoint4_state:{joint4_state:.3f}\
           \ncartesian_coordinate: x:{x:.3f}, y:{y:.3f}, z:{z:.3f}, phi:{phi:.3f}\n')

def update_joint_state(message: str):
    global joint1_state, joint2_state, joint3_state, joint4_state
    
    joint_states = [joint1_state, joint2_state, joint3_state, joint4_state]
    
    # 映射的范围和分割数
    ranges = [(-3, 3), (-0.13, 0.18), (-1.4, 1.4), (-3.15, 3.15)]
    num_parts = 180
    
    if message.startswith('joint_') and message.endswith(('increase', 'decrease')):
        joint_idx = int(message.split('_')[1]) - 1
        is_increase = message.endswith('increase')
        
        # 计算每份的大小
        part_size = (ranges[joint_idx][1] - ranges[joint_idx][0]) / num_parts
        
        # 确定增加还是减少
        step = 1 if is_increase else -1
        
        # 更新关节状态值
        new_state = joint_states[joint_idx] + step * part_size
        if is_increase:
            joint_states[joint_idx] = min(new_state, ranges[joint_idx][1])
        else:
            joint_states[joint_idx] = max(new_state, ranges[joint_idx][0])
        
        # 更新全局变量
        joint1_state, joint2_state, joint3_state, joint4_state = joint_states

    elif message == 'reset':
        joint1_state = 0.
        joint2_state = 0.
        joint3_state = 0.
        joint4_state = 0.

    print(f'Updated joint states: \njoint1_state:{joint1_state}, \njoint2_state:{joint2_state}, \njoint3_state:{joint3_state}, \njoint4_state:{joint4_state}')

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    print(msg)
    settings = saveTerminalSettings()
    rclpy.init()

    node = rclpy.create_node('keyboard_control')

    pub = node.create_publisher(String, 'my_joint_states', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    string_message = String()

    try:
        while True:
            key = getKey(settings)
            print(key)
            if key in jointBindings.keys():
                print(f'key:{key}, message:{jointBindings[key]}')
                update_joint_state(message=jointBindings[key])
            elif key in moveBindings.keys():
                print(f'key:{key}, message:{moveBindings[key]}')
                update_cartesian_state(message=moveBindings[key])
            elif (key == '\x03'):
                break
            else:
                print(f'undefined message')
            string_message.data = f'joint1_state:{joint1_state},joint2_state:{joint2_state},joint3_state:{joint3_state},joint4_state:{joint4_state},'
            pub.publish(string_message)


    except Exception as e:
        print(e)

    finally:
        string_message.data = f'joint1_state:{joint1_state},joint2_state:{joint2_state},joint3_state:{joint3_state},joint4_state:{joint4_state},'
        pub.publish(string_message)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()