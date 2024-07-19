import curses, threading, sys, queue
import rclpy
from nav_msgs.msg import Odometry
from runner import HdxNode, drain_queue, spin_thread


class OdometrySubscriber(HdxNode):
    def __init__(self, pos_queue: queue.Queue, namespace: str = ""):
        super().__init__('odometry_subscriber', namespace)
        self.subscribe_odom(self.listener_callback)
        self.pos_queue = pos_queue

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg)


def print_odometry(stdscr, msg: Odometry):
    p = msg.pose.pose.position
    h = msg.pose.pose.orientation
    stdscr.addstr(2, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    stdscr.addstr(3, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")


def main(stdscr):
    robot = sys.argv[1]
    stdscr.nodelay(True)
    stdscr.clear()

    finished = threading.Event()
    ros_ready = threading.Event()
    pos_queue = queue.Queue()
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: OdometrySubscriber(pos_queue, robot)))
    st.start()

    stdscr.addstr(0, 0, 'Enter "q" to quit')
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            if k == 'q':
                break
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(5, 0, traceback.format_exc())

        if ros_ready.is_set():
            stdscr.addstr(4, 0, "ROS2 ready")

        p = drain_queue(pos_queue)
        if p:
            print_odometry(stdscr, p)
        stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 odom_viewer.py robot")
    else:
        curses.wrapper(main)