import morph_contour_demo, curses_vision_demo
import curses, threading
import cv2
from queue import Queue


def main(stdscr):
    curses.curs_set(0)
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_YELLOW)
    curses.init_pair(2, curses.COLOR_BLUE, curses.COLOR_RED)
    stdscr.clear()

    running = threading.Event()
    running.set()
    image_queue = Queue()

    capture_thread = threading.Thread(target=curses_vision_demo.video_capture, args=(running, image_queue, 0),
                                      daemon=True)
    capture_thread.start()

    groundline_thread = threading.Thread(target=process_groundline, args=(running, (11, 11), 10, image_queue, stdscr),
                                         daemon=True)
    groundline_thread.start()

    stdscr.getch()
    running.clear()

    capture_thread.join()
    groundline_thread.join()


def process_groundline(running, kernel_size: int, min_space_width: int, image_queue: Queue, stdscr):
    while running.is_set():
        frame = image_queue.get()
        while not image_queue.empty():
            frame = image_queue.get()
        orig_height, orig_width = frame.shape[:2]
        contours, close_contour, best = contour_inner_loop(frame, kernel_size, min_space_width)
        stdscr.addstr(2, 0, "text")
        stdscr.addstr(4, 0, f"contour type: {type(close_contour)}")
        stdscr.addstr(3, 0, f"contour shape: {close_contour.shape}")
        stdscr.addstr(5, 0, f"{close_contour}")
        height, width = stdscr.getmaxyx()
        frame = cv2.resize(frame, (width, height))

        close_points = set()
        # inspiration from https://www.perplexity.ai/search/write-a-numpy-for-loop-examini-OgeoZY5MSwWrC7NRuLs7jw
        for i in range(close_contour.shape[0]):
            p = close_contour[i, 0]
            x_down = int(p[0] * width / orig_width)
            y_down = int(p[1] * height / orig_height)
            close_points.add((x_down, y_down))


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


def contour_inner_loop(frame, kernel_size, min_space_width):
    contours, hierarchy = morph_contour_demo.find_contours(frame, kernel_size)
    close_contour = morph_contour_demo.find_close_contour(contours, frame.shape[0])
    if close_contour is None:
        return contours, None, None
    else:
        clusters = morph_contour_demo.find_contour_clusters(close_contour)
        best = morph_contour_demo.best_contour_cluster(clusters, min_space_width)
        return contours, close_contour, best


if __name__ == '__main__':
    curses.wrapper(main)
