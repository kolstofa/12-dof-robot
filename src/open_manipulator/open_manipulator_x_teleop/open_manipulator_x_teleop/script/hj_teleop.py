from concurrent.futures import ThreadPoolExecutor
from math import exp
import os
import select
import sys
import rclpy

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
# from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from threading import Timer

# May 10 by Hojin
import socket

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# May 09 by Hojin
present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.1  # meter
joint_angle_delta = 0.5  # radian
# task_position_delta = 0.01  # meter
# joint_angle_delta = 0.05  # radian
path_time = 0.5  # second

usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)
- Gripper: Open     (G),    Close (F)

INIT : (1)
HOME : (2)

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8', 'joint9', 'joint10', 'joint11', 'joint12']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0],
                                                             goal_joint_angle[1],
                                                             goal_joint_angle[2],
                                                             goal_joint_angle[3],
                                                             goal_joint_angle[4],
                                                             goal_joint_angle[5],
                                                             goal_joint_angle[6],
                                                             goal_joint_angle[7],
                                                             goal_joint_angle[8],
                                                             goal_joint_angle[9],
                                                             goal_joint_angle[10],
                                                             goal_joint_angle[11]]
        self.goal_joint_space_req.path_time = path_time

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        print(present_joint_angle.count)

        # May 09 by Hojin
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]
        present_joint_angle[5] = msg.position[5]
        present_joint_angle[6] = msg.position[6]
        present_joint_angle[7] = msg.position[7]
        present_joint_angle[8] = msg.position[8]
        present_joint_angle[9] = msg.position[9]
        present_joint_angle[10] = msg.position[10]
        present_joint_angle[11] = msg.position[11]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 12):      # May 09 by Hojin
                goal_joint_angle[index] = present_joint_angle[index]


    def Uniry_ROS_PositionMove(self, config):
        self.goal_joint_space_req.joint_position.joint_name = [ 'joint1',
                                                                'joint2',
                                                                'joint3',
                                                                'joint4',
                                                                'joint5',
                                                                'joint6',
                                                                'joint21',
                                                                'joint22',
                                                                'joint23',
                                                                'joint24',
                                                                'joint25',
                                                                'joint26']
        self.goal_joint_space_req.joint_position.position = [float(config[0]),
                                                            float(config[1]),
                                                            float(config[2]),
                                                            float(config[3]),
                                                            float(config[4]),
                                                            float(config[5]),
                                                            float(config[6]),
                                                            float(config[7]),
                                                            float(config[8]),
                                                            float(config[9]),
                                                            float(config[10]),
                                                            float(config[11])]
        self.goal_joint_space_req.path_time = float(config[12])

        try:
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print_present_values()
    return key

def print_present_values():
    print(usage)

    # May 09 by Hojin
    print('Joint Angle(Rad): [{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4],
        present_joint_angle[5],
        present_joint_angle[6],
        present_joint_angle[7],
        present_joint_angle[8],
        present_joint_angle[9],
        present_joint_angle[10],
        present_joint_angle[11],
        ))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))

def serverSettings(HOST, PORT):
    # May 10 by Hojin
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen()

    client_socket, addr = server_socket.accept()
    print('Connected by', addr)

    return server_socket, client_socket

def Receive_Msg(sock):
    # May 10 by Hojin
    buf = b''
    length = 300

    return sock.recv(length)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    # May 10 by Hojin
    HOST = '127.0.0.1'
    PORT = 8101

    server_socket, client_socket = serverSettings(HOST, PORT)

    while True:
        byte_result = b""
        byte_result = Receive_Msg(client_socket)
        result = byte_result.decode('utf-8')

        print(result)

        result = result.split()

        if result[0] == "pose":
            teleop_keyboard.Uniry_ROS_PositionMove(result[1:])
        
        if result[0] == "end":
            print("Closing a Server...")
            break

        
    

    
    server_socket.close()
    client_socket.close()    


if __name__ == '__main__':
    main()