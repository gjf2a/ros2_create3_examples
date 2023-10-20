import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data

from queue import Queue
import threading

import cv2
import numpy as np
import remote_vision

class OrbDemo(runner.OpenCvCode):
    def __init__(self, msg_queue, max_match_distance=10):
        super().__init__(0, self.find_orb, (), msg_queue)
        self.orb = cv2.ORB_create(nfeatures=2000, nlevels=16, edgeThreshold=5)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.prev_kp = None
        self.prev_des = None
        self.max_match_distance = max_match_distance

    def find_orb(self, frame, cap, other):
        frame = cv2.resize(frame, (320, 240))
        kp, des = self.orb.detectAndCompute(frame, None)
        print(len(kp))
        if self.prev_kp is not None:
            matches = sorted([(kp[m.queryIdx].pt, self.prev_kp[m.trainIdx].pt) for m in self.matcher.match(des, self.prev_des) if m.distance < self.max_match_distance], key=lambda p: p[0][0])
            for ((x1, y1), (x2, y2)) in matches:
                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1)
            for i in range(1, len(matches)):
                x1 = int(matches[i - 1][0][0])
                y1 = int(matches[i - 1][0][1])
                x2 = int(matches[i][0][0])
                y2 = int(matches[i][0][1])
                #cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
            
        self.prev_kp = kp
        self.prev_des = des
        return frame, []


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: remote_vision.RemoteBot(msg_queue, f'/{sys.argv[1]}'), OrbDemo(msg_queue))
