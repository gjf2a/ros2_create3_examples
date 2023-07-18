import time
import threading
import sys
import rclpy
from rclpy.node import Node
import cv2
import math

from geometry_msgs.msg import Twist


def straight_twist(vel):
    t = Twist()
    t.linear.x = vel
    return t


def turn_twist(vel):
    t = Twist()
    t.angular.z = vel
    return t


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
            print(f"ROS2 active at {self.first_callback_time}")

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
