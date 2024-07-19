import curses, threading, queue, sys
import rclpy
from nav_msgs.msg import Odometry
from runner import GoToNode, drain_queue, spin_thread


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
    active = threading.Event()
    pos_queue = queue.Queue()
    cmd_queue = queue.Queue()
    status_queue = queue.Queue()
    current_input = ''
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: GoToNode(pos_queue, cmd_queue, status_queue, active, robot)))
    st.start()

    stdscr.addstr(0, 0, '"quit" to quit, "stop" to stop, "go x1 y1 [x2 y2...]" to drive somewhere')
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            curses.flushinp()
            if k == '\n':
                if current_input == 'quit':
                    break
                elif current_input == 'stop':
                    drain_queue(cmd_queue)
                    active.clear()
                elif current_input == 'reset':
                    drain_queue(cmd_queue)
                    active.clear()
                    cmd_queue.put('reset')
                elif current_input.startswith("go"):
                    coords = [float(n) for n in current_input.split()[1:]]
                    for i in range(0, len(coords), 2):
                        cmd_queue.put((coords[i], coords[i + 1]))
                else:
                    stdscr.addstr(5, 0, f'Unrecognized input: "{current_input}"')
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
            print_odometry(stdscr, p)

        s = drain_queue(status_queue)
        if s:
            stdscr.addstr(6, 0, f"{s}                                                ")
        stdscr.addstr(1, 0, f"{current_input}                         ")
        stdscr.addstr(7, 0, f"{'active  ' if active.is_set() else 'inactive'}")
        stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 go_to.py robot")
    else:
        curses.wrapper(main)
