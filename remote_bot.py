from curses import wrapper, curs_set
import threading
from queue import Queue
import sys
import rclpy
from runner import RemoteNode


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
    
    st = threading.Thread(target=spin_thread, args=(finished, lambda: RemoteNode(cmd_queue, pos_queue, f"/{sys.argv[1]}")))
    st.start()

    stdscr.addstr(1, 0, 'Enter "q" to quit')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif not cmd_queue.full():
            cmd_queue.put(k)
    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_bot robot_name")
    else:
        wrapper(main)
