from curses import wrapper, curs_set
import threading
from queue import Queue
import sys, time

from runner import RemoteNode, drain_queue
import rclpy
from geometry_msgs.msg import Pose

from w1thermsensor import W1ThermSensor
from w1thermsensor.units import Unit


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
    bump_queue = Queue()
    ir_queue = Queue()
    temperature = W1ThermSensor()
    bump_list = []
    
    st = threading.Thread(target=spin_thread, args=(finished, lambda: RemoteNode(cmd_queue, pos_queue, ir_queue, bump_queue, sys.argv[1])))
    st.start()

    temperature_filename = sys.argv[2]

    stdscr.addstr(1, 0, f'WASD to move; Q to quit; temperatures to {temperature_filename}')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif not cmd_queue.full():
            cmd_queue.put(k)

        pose = drain_queue(pos_queue)
        if pose:
            if type(pose) == str:
                stdscr.addstr(6, 0, f"str: {pose}")
            if type(pose) == Pose:
                p = pose.position
                h = pose.orientation
                stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
                current_temperature = temperature.get_temperature(Unit.DEGREES_F)
                stdscr.addstr(5, 0, f"Temperature: {current_temperature}F")
                with open(temperature_filename, 'a') as fout:
                    try:
                        fout.write(f"{current_temperature} {p} {time.time()}\n")
                    except:
                        print(f"update failed at {time.time()} {p}")
            elif type(pose) == float:
                stdscr.addstr(2, 0, f"{pose:.2f}")
            else:
                stdscr.addstr(8, 0, f"{type(pose)} {pose}")

        ir = drain_queue(ir_queue)
        if ir:
            stdscr.addstr(7, 0, f"ir: {ir}{' ' * 30}")

        if not bump_queue.empty():
            b = bump_queue.get()
            bump_list.append(b)
            stdscr.addstr(9, 0, f"{bump_list}")
        stdscr.refresh()

    finished.set()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: remote_bot robot_name temperature_filename")
    else:
        wrapper(main)
