import curses, sys, threading
from queue import Queue
import remote_wanderer, runner, curses_vision_demo


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

    robot_thread = threading.Thread(target=runner.spin_thread_recursive_node,
                                    args=(running, lambda: remote_wanderer.RemoteWandererNode(cmd_queue, pos_queue,
                                                                                              ir_queue, bump_queue,
                                                                                              sys.argv[1])))
    robot_thread.start()

    capture_thread = threading.Thread(target=curses_vision_demo.video_capture, args=(running, image_queue, 0), daemon=True)
    capture_thread.start()

    height, width = stdscr.getmaxyx()
    info_window = curses.newwin(8, width, 0, 0)
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
        curses_vision_demo.display_frame(frame, image_window)
        image_window.refresh()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_wanderer_video robot_name")
    else:
        curses.wrapper(main)