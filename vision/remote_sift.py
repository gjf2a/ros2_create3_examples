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

class SiftDemo(runner.OpenCvCode):
    def __init__(self, msg_queue):
        super().__init__(0, self.find_sift, (), msg_queue)
        self.sift = cv2.SIFT_create()

    def find_sift(self, frame, cap, other):
        frame = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp = self.sift.detect(gray, None)
        kp, des = self.sift.compute(gray, kp)
        print(len(kp))
        keypoint_img = cv2.drawKeypoints(gray, kp, None, color=(0, 255, 0), flags=0)
        return keypoint_img, []


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: remote_vision.RemoteBot(msg_queue, f'/{sys.argv[1]}'), SiftDemo(msg_queue))
