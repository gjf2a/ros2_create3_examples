# From https://www.perplexity.ai/search/write-a-python-ZqWIYEsXTJiLiru0F7XzIg

import cv2
import curses
import time
import numpy as np

colors = [
        (curses.COLOR_BLACK,   (  0,   0,   0)),
        (curses.COLOR_RED,     (255,   0,   0)), 
        (curses.COLOR_GREEN,   (  0, 255,   0)), 
        (curses.COLOR_YELLOW,  (255, 255,   0)), 
        (curses.COLOR_BLUE,    (  0,   0, 255)), 
        (curses.COLOR_MAGENTA, (255,   0, 255)), 
        (curses.COLOR_CYAN,    (  0, 255, 255)), 
        (curses.COLOR_WHITE,   (255, 255, 255))
]


def main(stdscr):
    # Initialize the webcam
    cap = cv2.VideoCapture(0)

    # Get the screen dimensions
    height, width = stdscr.getmaxyx()

    # Create a window to display the webcam feed
    win = curses.newwin(height, width, 0, 0)

    for i, color in enumerate(colors):
        if i > 0:
            curses.init_pair(i, curses.COLOR_BLACK, color[0])

    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

        # Resize the frame to fit the window
        frame = cv2.resize(frame, (width, height))

        # Update the window with the grayscale frame
        for y in range(height - 1):
            for x in range(width):
                win.addch(y, x, '.', curses.color_pair(color_from(frame[y, x])))    

        # Refresh the window
        win.refresh()

        # Wait for a short time to control the frame rate
        #time.sleep(0.1)

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
