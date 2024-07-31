import threading, curses
from queue import Queue
import sys, time

from runner import drain_queue, spin_thread_recursive_node
from remote_wanderer import RemoteWandererNode
from curses_vision_demo import video_capture, display_frame

from geometry_msgs.msg import Pose

from w1thermsensor import W1ThermSensor
from w1thermsensor.units import Unit


def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    height, width = stdscr.getmaxyx()
    info_window = curses.newwin(10, width, 0, 0)
    image_window = curses.newwin(height - 12, width, 12, 0)

    running = threading.Event()
    running.set()
    cmd_queue = Queue()
    pos_queue = Queue()
    bump_queue = Queue()
    ir_queue = Queue()
    image_queue = Queue()
    temperature = W1ThermSensor()
    bump_list = []
    
    st = threading.Thread(target=spin_thread_recursive_node,
                          args=(running, lambda: RemoteWandererNode(cmd_queue, pos_queue, ir_queue, bump_queue,
                                                                    sys.argv[1])))
    st.start()

    capture_thread = threading.Thread(target=video_capture, args=(running, image_queue, 0), daemon=True)
    capture_thread.start()

    temperature_filename = sys.argv[2]

    info_window.addstr(1, 0, f'WASD to move; F to Freely Wander; Q to quit; temperatures to {temperature_filename}')
    info_window.refresh()
    
    while running.is_set():
        try:
            k = info_window.getkey()
            if k == 'q':
                running.clear()
            elif not cmd_queue.full():
                cmd_queue.put(k)
        except curses.error:
            pass

        pose = drain_queue(pos_queue)
        if pose:
            if type(pose) == str:
                info_window.addstr(6, 0, f"str: {pose}")
            if type(pose) == Pose:
                p = pose.position
                h = pose.orientation
                info_window.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                info_window.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
                current_temperature = temperature.get_temperature(Unit.DEGREES_F)
                info_window.addstr(5, 0, f"Temperature: {current_temperature}F")
                with open(temperature_filename, 'a') as fout:
                    try:
                        fout.write(f"{current_temperature} {p} {time.time()}\n")
                    except:
                        print(f"update failed at {time.time()} {p}")
            elif type(pose) == float:
                info_window.addstr(2, 0, f"{pose:.2f}")
            else:
                info_window.addstr(8, 0, f"{type(pose)} {pose}")

        ir = drain_queue(ir_queue)
        if ir:
            info_window.addstr(7, 0, f"ir: {ir}{' ' * 30}")

        if not bump_queue.empty():
            b = bump_queue.get()
            bump_list.append(b)
            info_window.addstr(9, 0, f"{bump_list}")
        info_window.refresh()

        frame = drain_queue(image_queue)
        if frame is not None:
            display_frame(frame, image_window)
            image_window.refresh()

    capture_thread.join()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: remote_curses_temp robot_name temperature_filename")
    else:
        curses.wrapper(main)
