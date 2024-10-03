import morph_contour_demo, curses_vision_demo
import curses, threading, time
import cv2
from queue import Queue
from typing import Set, Tuple


DATA_BLOCK_ROWS = 6


def main(stdscr):
    curses.curs_set(0)
    init_groundline_colors()
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


def init_groundline_colors():
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_YELLOW)
    curses.init_pair(2, curses.COLOR_BLUE, curses.COLOR_RED)


def process_groundline(running, kernel_size: Tuple[int,int], min_space_width: int, image_queue: Queue, stdscr):
    start = time.time()
    num_frames = 0
    while running.is_set():
        frame = image_queue.get()
        while not image_queue.empty():
            frame = image_queue.get()
        if frame is not None:
            orig_height, orig_width = frame.shape[:2]
            contours, close_contour, best = contour_inner_loop(frame, kernel_size, min_space_width)
            if close_contour is not None:
                stdscr.addstr(2, 0, "text")
                stdscr.addstr(3, 0, f"contour shape: {close_contour.shape}")
                stdscr.addstr(4, 0, f"contour type: {type(close_contour)}")
            elapsed = time.time() - start
            stdscr.addstr(5, 0, f"{num_frames/elapsed:.1f} fps ({num_frames}/{elapsed:.1f}s)")
            height, width = stdscr.getmaxyx()
            height -= DATA_BLOCK_ROWS
            frame = cv2.resize(frame, (width, height))

            close_points = extract_reduced_points(close_contour, orig_width, orig_height, width, height)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for y in range(height - 1):
                for x in range(width):
                    if (x, y) in close_points:
                        stdscr.addch(y + DATA_BLOCK_ROWS, x, 'C', curses.color_pair(1))
            #        elif (x_up, y_up) in contours:
            #            stdscr.addch(y, x, 'c', curses.color_pair(2))
                    else:
                        stdscr.addch(y + DATA_BLOCK_ROWS, x, curses_vision_demo.gray2char(gray[y, x]))
            num_frames += 1
            stdscr.refresh()


def extract_reduced_points(numpy_points, orig_width, orig_height, reduced_width, reduced_height) -> Set[Tuple[int,int]]:
    # inspiration from https://www.perplexity.ai/search/write-a-numpy-for-loop-examini-OgeoZY5MSwWrC7NRuLs7jw
    reduced = set()
    if numpy_points is not None:
        for i in range(numpy_points.shape[0]):
            p = numpy_points[i, 0]
            x_down = int(p[0] * reduced_width / orig_width)
            y_down = int(p[1] * reduced_height / orig_height)
            reduced.add((x_down, y_down))
    return reduced


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
