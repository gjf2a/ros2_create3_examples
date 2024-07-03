from curses import wrapper, curs_set
import threading
from queue import Queue
import sys

from runner import RemoteNode, drain_queue
import rclpy
from geometry_msgs.msg import Pose


def spin_thread(finished, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
    node.reset()
    rclpy.shutdown()


def main(stdscr):
    curs_set(0)
    stdscr.clear()

    finished = threading.Event()
    cmd_queue = Queue()
    pos_queue = Queue()
    
    st = threading.Thread(target=spin_thread, args=(finished, lambda: RemoteNode(cmd_queue, pos_queue, sys.argv[1])))
    st.start()

    stdscr.addstr(1, 0, 'WASD to move; Q to quit')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif not cmd_queue.full():
            cmd_queue.put(k)
        pose = drain_queue(pos_queue)
        if pose:
            if type(pose) == Pose:
                p = pose.position
                h = pose.orientation
                stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
            elif type(pose) == float:
                stdscr.addstr(2, 0, f"{pose:.2f}")
            elif type(pose) == list:
                stdscr.addstr(6, 0, f"{pose}")
            else:
                stdscr.addstr(5, 0, f"{type(pose)} {pose}")
        stdscr.refresh()

    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_bot robot_name")
    else:
        wrapper(main)
