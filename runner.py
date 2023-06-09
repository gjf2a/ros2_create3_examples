import time
import threading
import sys
import rclpy
from rclpy.node import Node


class HdxNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.start = time.time()
        self.first_callback_time = None

    def record_first_callback(self):
        if self.first_callback_time is None:
            self.first_callback_time = self.elapsed_time()
            print(f"ROS2 active at {self.first_callback_time}")

    def ros_issuing_callbacks(self):
        return self.first_callback_time is not None

    def elapsed_time(self):
        return time.time() - self.start

    def reset(self):
        pass


def run_single_node(node_maker):
    finished = threading.Event()
    ros_ready = threading.Event()
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, node_maker))
    it = threading.Thread(target=input_thread, args=(finished, ros_ready))
    it.start()
    st.start()
    it.join()
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
    while executor.context.ok() and not finished.is_set():
        executor.spin_once()
        if node.ros_issuing_callbacks():
            ros_ready.set()
    node.reset()
    rclpy.shutdown()
