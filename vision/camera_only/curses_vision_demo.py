# Before running this program, type the following command:
# sudo chmod 666 /dev/video0
# Otherwise, it will fail due to lack of permission.
# Maybe I should just add it to this program! But then you have to sudo...

# From https://www.perplexity.ai/search/write-a-python-ZqWIYEsXTJiLiru0F7XzIg

import cv2
import curses
import time
import numpy as np
import queue
import threading
import sys


def video_capture(running, image_queue, port: int):
    cap = cv2.VideoCapture(port)
    while running.is_set():
        ret, frame = cap.read()
        image_queue.put(frame)


encodings = [' ', '.', ':', ';', '!', '?', '+', '*', '@', '#']


def gray2char(gray):
    gap = 1 + (255 // len(encodings))
    return encodings[gray // gap]


def video_display(event, image_queue, win):
    start = None

    frames_acquired = 0
    frames_displayed = 0

    while event.is_set():
        frame = image_queue.get()
        if start is None:
            start = time.time()
        frames_acquired += 1
        while not image_queue.empty():
            frame = image_queue.get()
            frames_acquired += 1

        display_frame(frame, win)

        frames_displayed += 1
        elapsed = time.time() - start
        win.addstr(0, 0, f"{frames_displayed / elapsed:.2f} hz ({frames_acquired / elapsed:.2f} hz)")

        # Refresh the window
        win.refresh()


def display_frame(frame, win):
    height, width = win.getmaxyx()
    frame = cv2.resize(frame, (width, height))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    for y in range(height - 1):
        for x in range(width):
            win.addch(y, x, gray2char(gray[y, x]))



def main(stdscr):
    event = threading.Event()
    image_queue = queue.Queue()
    event.set()

    capture_thread = threading.Thread(target=video_capture, args=(event, image_queue, 0), daemon=True)
    display_thread = threading.Thread(target=video_display, args=(event, image_queue, stdscr), daemon=True)
    capture_thread.start()
    display_thread.start()

    stdscr.getch()
    event.clear()    

    capture_thread.join()
    display_thread.join()
    
    

def euclidean_distance(a, b):
    assert len(a) == len(b)
    return sum((a[i] - b[i])**2 for i in range(len(a)))


def color_from(triple):
    best = None
    best_dist = None
    for i, (color, codes) in enumerate(colors):
        dist = euclidean_distance(codes, triple)
        if best_dist is None or dist < best_dist:
            best = i
            best_dist = dist
    return best


if __name__ == "__main__":
    # Initialize the curses screen
    curses.wrapper(main)
