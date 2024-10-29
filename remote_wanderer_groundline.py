# Before running this program, type the following command:
# sudo chmod 666 /dev/video0


import curses, sys, threading
from queue import Queue
import remote_wanderer, runner, curses_vision_demo, groundline_video
from typing import Tuple
import cv2


def main(stdscr):
    curses.curs_set(0)
    groundline_video.init_groundline_colors()
    stdscr.clear()

    running = threading.Event()
    running.set()
    cmd_queue = Queue()
    pos_queue = Queue()
    bump_queue = Queue()
    ir_queue = Queue()
    image_queue = Queue()
    bump_list = []

    robot_thread = threading.Thread(target=runner.spin_thread_recursive_node,
                                    args=(running, lambda: remote_wanderer.RemoteWandererNode(cmd_queue, pos_queue,
                                                                                              ir_queue, bump_queue,
                                                                                              sys.argv[1])))
    robot_thread.start()

    capture_thread = threading.Thread(target=curses_vision_demo.video_capture, args=(running, image_queue, 0), daemon=True)
    capture_thread.start()

    height, width = stdscr.getmaxyx()
    info_window = curses.newwin(9, width, 0, 0)
    image_window = curses.newwin(height - 10, width, 10, 0)

    stdscr.nodelay(True)
    info_window.addstr(1, 0, 'WASD to move; F to Freely Wander; Q to quit')
    info_window.refresh()

    while running.is_set():
        remote_wanderer.get_cmd(stdscr, cmd_queue, running)

        handle_image(image_queue, image_window)

        remote_wanderer.display_pose(info_window, pos_queue, 2)
        remote_wanderer.display_ir(info_window, ir_queue, 7)
        remote_wanderer.display_bump(info_window, bump_queue, bump_list, 8)
        info_window.refresh()

    capture_thread.join()
    robot_thread.join()


def handle_image(image_queue, image_window):
    frame = runner.drain_queue(image_queue)
    if frame is not None:
        process_groundline(frame, (11, 11), 10, image_window)


def process_groundline(frame, kernel_size: Tuple[int,int], min_space_width: int, stdscr):
    orig_height, orig_width = frame.shape[:2]
    contours, close_contour, best = groundline_video.contour_inner_loop(frame, kernel_size, min_space_width)
    height, width = stdscr.getmaxyx()
    frame = cv2.resize(frame, (width, height))

    close_points = groundline_video.extract_reduced_points(close_contour, orig_width, orig_height, width, height)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for y in range(height - 1):
        for x in range(width):
            if (x, y) in close_points:
                stdscr.addch(y, x, 'C', curses.color_pair(1))
    #        elif (x_up, y_up) in contours:
    #            stdscr.addch(y, x, 'c', curses.color_pair(2))
            else:
                stdscr.addch(y, x, curses_vision_demo.gray2char(gray[y, x]))
    stdscr.refresh()



if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_wanderer_groundline robot_name")
    else:
        curses.wrapper(main)
