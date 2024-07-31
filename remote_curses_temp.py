# Before running this program, type the following command:
# sudo chmod 666 /dev/video0

import curses, sys, threading, time
from queue import Queue
import remote_wanderer, runner, curses_vision_demo, remote_wanderer_video
from w1thermsensor import W1ThermSensor
from w1thermsensor.units import Unit


def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    running = threading.Event()
    running.set()
    cmd_queue = Queue()
    pos_queue = Queue()
    bump_queue = Queue()
    ir_queue = Queue()
    image_queue = Queue()
    bump_list = []
    thermometer = W1ThermSensor()

    robot_thread = threading.Thread(target=runner.spin_thread_recursive_node,
                                    args=(running, lambda: remote_wanderer.RemoteWandererNode(cmd_queue, pos_queue,
                                                                                              ir_queue, bump_queue,
                                                                                              sys.argv[1])))
    robot_thread.start()

    capture_thread = threading.Thread(target=curses_vision_demo.video_capture, args=(running, image_queue, 0), daemon=True)
    capture_thread.start()

    height, width = stdscr.getmaxyx()
    info_window = curses.newwin(11, width, 0, 0)
    image_window = curses.newwin(height - 12, width, 12, 0)

    stdscr.nodelay(True)
    info_window.addstr(1, 0, 'WASD to move; F to Freely Wander; Q to quit')
    info_window.refresh()

    while running.is_set():
        remote_wanderer.get_cmd(stdscr, cmd_queue, running)

        remote_wanderer_video.handle_image(image_queue, image_window)

        pose = remote_wanderer.display_pose(info_window, pos_queue, 2)
        remote_wanderer.display_ir(info_window, ir_queue, 7)
        remote_wanderer.display_bump(info_window, bump_queue, bump_list, 8)
        handle_temperature(info_window, thermometer, 9, pose)
        info_window.refresh()

    capture_thread.join()
    robot_thread.join()


def handle_temperature(stdscr, thermometer, line, pos):
    current_temperature = thermometer.get_temperature(Unit.DEGREES_F)
    stdscr.addstr(line, 0, f"Temperature: {current_temperature}F")
    with open(sys.argv[3], 'a') as fout:
        try:
            fout.write(f"{current_temperature} {p} {time.time()}\n")
        except:
            print(f"update failed at {time.time()} {p}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: remote_curses_temp robot_name temperature_filename")
    else:
        curses.wrapper(main)
