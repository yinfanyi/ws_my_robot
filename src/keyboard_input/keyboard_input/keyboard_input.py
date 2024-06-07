from rclpy.node import Node
from std_msgs.msg import String
from math import sin, cos, pi

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
c 改变工作模式，默认关节空间，按下切换到末端位姿空间
方向键控制末端位姿空间前后左右
  ↑
← ↓ → 
z x 控制末端位姿空间上下
ctrl-c to break
"""
moveBindings = {
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

joint1_state = 0.
joint2_state = 0.
joint3_state = 0.
joint4_state = 0.

def change_joint_state(message: str):
    global joint1_state, joint2_state, joint3_state, joint4_state
    
    joint_states = [joint1_state, joint2_state, joint3_state, joint4_state]
    
    # 映射的范围和分割数
    ranges = [(-3, 3), (-0.13, 0.18), (-1.4, 1.4), (-3.15, 3.15)]
    num_parts = 10
    
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
            if key in moveBindings.keys():
                print(f'key:{key}, message:{moveBindings[key]}')
                change_joint_state(message=moveBindings[key])
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