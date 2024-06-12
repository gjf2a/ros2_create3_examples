import time
import threading
import sys
import rclpy
from rclpy.node import Node
import cv2
import math

from geometry_msgs.msg import Twist, Quaternion
from irobot_create_msgs.msg import WheelStatus

from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_sensor_data

from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle, DriveDistance

from enum import Enum
from typing import Tuple


def drain_queue(q):
    result = None
    while not q.empty():
        result = q.get()
    return result



def straight_twist(vel):
    t = Twist()
    t.linear.x = vel
    return t


def turn_twist(vel):
    t = Twist()
    t.angular.z = vel
    return t


def quaternion2euler(orientation: Quaternion) -> Tuple[float, float, float]:
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw)
    roll is rotation around x-axis in radians (counterclockwise)
    pitch is rotation around y-axis in radians (counterclockwise)
    yaw is rotation around z-axis in radians (counterclockwise)
    """
    # Concept: https://www.perplexity.ai/search/explain-the-orientation-jXV4N3xiQUeUGwMTLESZXg

    w, x, y, z = orientation.x, orientation.y, orientation.z, orientation.w

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


BUMP_HEADINGS = {
    'bump_left': -math.pi / 2,
    'bump_front_left': -math.pi / 4,
    'bump_front_center': 0.0,
    'bump_front_right': math.pi / 4,
    'bump_right': math.pi / 2
}


def find_bump_from(detections):
    for detected in detections:
        if detected.header.frame_id.startswith('bump'):
            return detected.header.frame_id


class Timer:
    def __init__(self):
        self.start = time.time()
        self.count = 0

    def inc(self):
        self.count += 1

    def elapsed(self):
        return self.count / (time.time() - self.start)


class HdxNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.start = time.time()
        self.first_callback_time = None
        self.done = False

    def record_first_callback(self):
        if self.first_callback_time is None:
            self.first_callback_time = self.elapsed_time()
            print(f"First ROS2 callback for {self.get_name()} at {self.first_callback_time}")

    def ros_issuing_callbacks(self):
        return self.first_callback_time is not None

    def elapsed_time(self):
        return time.time() - self.start

    def quit(self):
        self.done = True

    def reset(self):
        pass

    def quitting(self):
        return self.done


class WheelMonitorNode(HdxNode):
    def __init__(self, name, namespace):
        super().__init__(name)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.last_wheel_status = None

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg


class RemoteNode(HdxNode):
    """
    ROS2 node that sends driving messages to the robot based on keyboard input,
    and sends back timing and odometry information.

    Attributes
    ----------
    cmd_queue - receives wasd movement commands
    pos_queue - sends odometry, timestamps, and keys it recognizes
    subscription - receives odometry information
    publisher - sends motor commands

    Methods
    -------
    listener_callback() - sends pose information when received
    timer_callback() - sends timing and key information at time intervals.
    """
    def __init__(self, cmd_queue, pos_queue, namespace: str = ""):
        super().__init__('remote_control_node')

        self.commands = {
            'w': straight_twist(0.5),
            'a': turn_twist(math.pi/4),
            's': straight_twist(0.0),
            'd': turn_twist(-math.pi/4)
        }

        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)
        self.cmd_queue = cmd_queue
        self.pos_queue = pos_queue

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg.pose.pose)

    def timer_callback(self):
        self.pos_queue.put(self.elapsed_time())
        msg = drain_queue(self.cmd_queue)
        if msg is not None and msg in self.commands:
            self.publisher.publish(self.commands[msg])
            # Send an echo so the sender knows the publish happened.
            self.pos_queue.put(msg) 


class GoToPhase(Enum):
    INACTIVE = 0
    AIMING = 1
    TRAVELING = 2


class GoToNode(HdxNode):
    """
    ROS2 node that awaits (x, y) coordinates to which to navigate. The sender is
    responsible for ensuring a clear path from the robot's current location to 
    the specificed position. It sends position data back as it receives it.
    It also maintains a condition variable, which is set when it is active,
    and clear when it is inactive. It becomes active when it receives a 
    command and inactive when it has reached its target destination.
    """
    def __init__(self, cmd_queue, pos_queue, is_active, namespace: str = ""):
        super().__init__('go_to_node')
        self.cmd_queue = cmd_queue
        self.pos_queue = pos_queue
        self.is_active = is_active

        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)

        self.phase = GoToPhase.INACTIVE
        self.last_pose = None
        self.goal_orientation = None
        self.goal_position = None

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg.pose.pose)
        self.last_pose = msg.pose.pose
        
    def timer_callback(self):
        msg = drain_queue(self.cmd_queue)
        if msg is not None and self.last_pose is not None:
            self.is_active.set()
            self.goal_position = msg
            x, y = msg
            self.goal_orientation = math.atan2(y - self.last_pose.position.y, x - self.last_pose.position.x)
            self.phase = GoToPhase.AIMING
            # TODO: Check best turn direction.
            #       Start turn.
            #       When orientation falls within a certain tolerance of the goal, shift to TRAVELING 



def run_single_node(node_maker):
    finished = threading.Event()
    ros_ready = threading.Event()
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, node_maker))
    it = threading.Thread(target=input_thread, args=(finished, ros_ready))
    it.start()
    st.start()
    it.join()
    st.join()


class CvKey:
    def __init__(self, key_ascii):
        self.key = chr(key_ascii)

    def is_quit(self):
        return self.key == 'q'


def package_keystroke(keystroke):
    keystroke = keystroke & 0xFF
    if keystroke < 0xFF:
        return CvKey(keystroke)


class OpenCvCode:
    def __init__(self, video_port, frame_proc, frame_proc_args, msg_queue):
        self.msg_queue = msg_queue
        self.video_port = video_port
        self.frame_proc = frame_proc
        self.frame_proc_args = frame_proc_args
        
    def loop(self, finished):
        print("Starting OpenCV loop")
        cap = cv2.VideoCapture(self.video_port)
        timer = Timer()
        while True:
            status, frame = cap.read()
            final_frame, other_data = self.frame_proc(frame, cap, self.frame_proc_args)
            cv2.imshow('OpenCV image', final_frame)
            timer.inc()

            if self.msg_queue.empty():
                self.msg_queue.put(other_data)

            key = package_keystroke(cv2.waitKey(1))
            if key:
                self.msg_queue.put(key)
                if key.is_quit():
                    finished.set()
                    break
        fps = timer.elapsed()
        cap.release()
        cv2.destroyAllWindows()
        print(f"FPS: {fps}")


def run_vision_node(node_maker, cv_object):
    finished = threading.Event()
    ros_ready = threading.Event()

    vt = threading.Thread(target=lambda cv: cv.loop(finished), args=(cv_object,))
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, node_maker))
    vt.start()
    st.start()
    vt.join()
    st.join()


def input_thread(finished, ros_ready):
    ros_ready.wait()
    user = input("Type anything to exit")
    finished.set()
    rclpy.shutdown()


def spin_thread(finished, ros_ready, node_maker):
    print("starting")
    rclpy.init(args=None)
    print("init done")
    
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    print("Node created and added")
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
        if node.ros_issuing_callbacks():
            ros_ready.set()
    node.reset()
    rclpy.shutdown()
    print("ROS2 has shut down")


# Inspired by: https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/?answer=377861#post-id-377861
# call rclpy.init() before invoking
def run_multiple_nodes(*nodes):
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    user = input("Type anything to exit")
    rclpy.shutdown()
    executor_thread.join()


# call rclpy.init() before invoking
def run_recursive_node(recursive_node):
    executor = rclpy.executors.MultiThreadedExecutor()
    recursive_node.add_self_recursive(executor)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    user = input("Type anything to exit")
    rclpy.shutdown()
    executor_thread.join()


# call rclpy.init() before invoking
def run_recursive_vision_node(cv_object, recursive_node):
    finished = threading.Event()
    vt = threading.Thread(target=lambda cv: cv.loop(finished), args=(cv_object,))
    vt.start()
    executor = rclpy.executors.MultiThreadedExecutor()
    recursive_node.add_self_recursive(executor)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    vt.join()
    rclpy.shutdown()
    executor_thread.join()


# call rclpy.init() before invoking
def run_vision_multiple_nodes(cv_object, *nodes):
    finished = threading.Event()
    vt = threading.Thread(target=lambda cv: cv.loop(finished), args=(cv_object,))
    vt.start()
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    vt.join()
    rclpy.shutdown()
    executor_thread.join()


class CustomActionClient(Node):
    def __init__(self, name, py_action_type, ros2_action_type, callback, namespace):
        super().__init__(name)
        self._action_client = ActionClient(self, py_action_type, f'{namespace}/{ros2_action_type}')
        self.callback = callback
        self.goal_handle = None

    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            print("Goal accepted.")
            self._get_result_future = self.goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.callback)
        else:
            print("Goal rejected...")

    def cancel(self):
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal()
            if future:
                print("Goal cancelled")
            else:
                print("Failed to cancel goal")

    def spin_thread(self):
        st = threading.Thread(target=lambda ac: rclpy.spin(ac), args=(self,))
        st.start()


class RotateActionClient(CustomActionClient):
    def __init__(self, callback, namespace):
        super().__init__("RotateActionClient", RotateAngle, 'rotate_angle', callback, namespace)
        
    def send_goal(self, goal_heading, radians_per_sec=1.0):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = goal_heading
        goal_msg.max_rotation_speed = radians_per_sec
        super().send_goal(goal_msg)


class DriveDistanceClient(CustomActionClient):
    def __init__(self, callback, namespace):
        super().__init__("DriveDistanceClient", DriveDistance, 'drive_distance', callback, namespace)

    def send_goal(self, goal_distance, meters_per_sec=0.3):
        goal_msg = DriveDistance.Goal()
        goal_msg.distance = goal_distance
        goal_msg.max_translation_speed = meters_per_sec
        super().send_goal(goal_msg)
