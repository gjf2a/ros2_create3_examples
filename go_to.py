import curses, threading, sys, math, queue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from runner import GoToNode, drain_queue

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


def printOdometry(stdscr, msg: Odometry):
    p = msg.pose.pose.position
    h = msg.pose.pose.orientation
    stdscr.addstr(2, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    stdscr.addstr(3, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")


def main(stdscr):
    stdscr.nodelay(True)
    stdscr.clear()

    finished = threading.Event()
    ros_ready = threading.Event()
    active = threading.Event()
    pos_queue = queue.Queue()
    cmd_queue = queue.Queue()
    status_queue = queue.Queue()
    current_input = ''
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: GoToNode(pos_queue, cmd_queue, status_queue, active, "/archangel")))
    st.start()

    stdscr.addstr(0, 0, '"quit" to quit, "go x y" to drive somewhere')
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            curses.flushinp()
            if k == '\n':
                if current_input == 'quit':
                    break
                elif current_input.startswith("go"):
                    x, y = [float(n) for n in current_input.split()[1:]]
                    cmd_queue.put((x, y))
                current_input = ''
            elif k == '\b':
                current_input = current_input[:-1]
            else:
                current_input += k
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(5, 0, traceback.format_exc())

        if ros_ready.is_set():
            stdscr.addstr(4, 0, "ROS2 ready")

        p = drain_queue(pos_queue)
        if p:
            printOdometry(stdscr, p)

        s = drain_queue(status_queue)
        if s:
            stdscr.addstr(6, 0, f"{s}                                                ")
        stdscr.addstr(1, 0, f"{current_input}                         ")
        stdscr.addstr(7, 0, f"{'active' if active.is_set() else 'inactive'}")
        stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    curses.wrapper(main)
