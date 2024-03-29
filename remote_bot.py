from curses import wrapper, curs_set
import threading
from queue import Queue
import sys
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from runner import HdxNode, straight_twist, turn_twist

def spin_thread(finished, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
    node.reset()
    rclpy.shutdown()


COMMANDS = {
    'w': straight_twist(0.5),
    'a': turn_twist(math.pi/4),
    's': straight_twist(0.0),
    'd': turn_twist(-math.pi/4)
}


class RemoteNode(HdxNode):
    def __init__(self, stdscr, msg_queue, namespace: str = ""):
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)
        self.msg_queue = msg_queue
        self.stdscr = stdscr
        self.last_key = None

    def listener_callback(self, msg: Odometry):
        self.printOdometry(msg)

    def timer_callback(self):
        self.stdscr.addstr(2, 0, f"{self.elapsed_time():7.2f} s")
        if not self.msg_queue.empty():
            msg = self.msg_queue.get()
            self.last_key = msg
            if self.last_key in COMMANDS:
                self.publisher.publish(COMMANDS[msg])
            self.stdscr.addstr(5, 0, f"{msg} ({self.last_key})")
        self.stdscr.refresh()

    def printOdometry(self, msg: Odometry):
        p = msg.pose.pose.position
        h = msg.pose.pose.orientation
        self.stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
        self.stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")
        self.stdscr.refresh()


def main(stdscr):
    curs_set(0)
    stdscr.clear()

    finished = threading.Event()
    msg_queue = Queue(maxsize=1)
    
    st = threading.Thread(target=spin_thread, args=(finished, lambda: RemoteNode(stdscr, msg_queue, f"/{sys.argv[1]}")))
    st.start()

    stdscr.addstr(1, 0, 'Enter "q" to quit')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif not msg_queue.full():
            msg_queue.put(k)
    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_bot robot_name")
    else:
        wrapper(main)
