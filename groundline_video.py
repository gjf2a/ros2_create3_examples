import morph_contour_demo, curses_vision_demo
import curses, threading
import cv2
from queue import Queue


def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    running = threading.Event()
    running.set()
    image_queue = Queue()
    groundline_queue = Queue()

    capture_thread = threading.Thread(target=curses_vision_demo.video_capture, args=(running, image_queue, 0),
                                      daemon=True)
    capture_thread.start()

    groundline_thread = threading.Thread(target=process_groundline, args=(running, 11, 10, image_queue, groundline_queue),
                                         daemon=True)
    groundline_thread.start()

    display_thread = threading.Thread(target=curses_vision_demo.video_display, args=(running, groundline_queue, stdscr),
                                      daemon=True)
    display_thread.start()

    stdscr.getch()
    running.clear()

    capture_thread.join()
    display_thread.join()
    groundline_thread.join()


def process_groundline(running, kernel_size: int, min_space_width: int, image_queue: Queue, groundline_queue: Queue):
    while running.is_set():
        frame = image_queue.get()
        while not image_queue.empty():
            frame = image_queue.get()
        contours, close_contour, best = contour_inner_loop(frame, kernel_size, min_space_width)
        cv2.drawContours(frame, close_contour, -1, (255, 255, 255), 3)
        groundline_queue.put(frame)


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