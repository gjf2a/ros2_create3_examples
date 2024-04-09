# From https://www.perplexity.ai/search/write-a-python-ZqWIYEsXTJiLiru0F7XzIg

import cv2
import curses
import time

def main(stdscr):
    # Initialize the webcam
    cap = cv2.VideoCapture(0)

    # Get the screen dimensions
    height, width = stdscr.getmaxyx()

    # Create a window to display the webcam feed
    win = curses.newwin(height, width, 0, 0)

    while True:
        # Capture a frame from the webcam
        ret, frame = cap.read()

        height -= 1

        # Resize the frame to fit the window
        frame = cv2.resize(frame, (width, height))

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Update the window with the grayscale frame
        for y in range(height):
            row = ""
            for x in range(width):
                row += chr(gray[y, x])
            win.addstr(y, 0, row)

        # Refresh the window
        win.refresh()

        # Wait for a short time to control the frame rate
        time.sleep(0.1)

if __name__ == "__main__":
    # Initialize the curses screen
    curses.wrapper(main)
