import sys
import cv2
import numpy as np
import time

from queue import Queue


class Timer:
    def __init__(self):
        self.start = time.time()
        self.count = 0

    def inc(self):
        self.count += 1

    def elapsed(self):
        return self.count / (time.time() - self.start)
    


def loop(video_port):
    cap = cv2.VideoCapture(video_port)
    timer = Timer()
    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame, (640, 480))

        # Display the resulting frame
        cv2.imshow('frame', frame)
        timer.inc()

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    fps = timer.elapsed()
    cap.release()
    cv2.destroyAllWindows()
    print("FPS:", fps)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: simple_opencv_demo.py video_port")
    else:
        loop(int(sys.argv[1]))

