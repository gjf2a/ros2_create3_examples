from curses import wrapper
import threading
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from runner import HdxNode

def spin_thread(finished, ros_ready, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
        if node.ros_issuing_callbacks():
            ros_ready.set()
    node.reset()
    rclpy.shutdown()


# Adapted from https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/sub_battery.py
class OdometrySubscriber(HdxNode):
    def __init__(self, stdscr, namespace: str = ""):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)
        self.stdscr = stdscr

    def listener_callback(self, msg: Odometry):
        self.printOdometry(msg)

    def printOdometry(self, msg: Odometry):
        p = msg.pose.pose.position
        h = msg.pose.pose.orientation
        self.stdscr.addstr(1, 0, f"Time:        {self.elapsed_time():7.2f} s")
        self.stdscr.addstr(2, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
        self.stdscr.addstr(3, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")
        self.stdscr.refresh()


def main(stdscr):
    stdscr.clear()

    finished = threading.Event()
    ros_ready = threading.Event()
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: OdometrySubscriber(stdscr, "/archangel")))
    st.start()

    stdscr.addstr(0, 0, 'Enter "q" to quit')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif ros_ready.is_set():
            stdscr.addstr(4, 0, "ROS2 ready")
            stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    wrapper(main)
