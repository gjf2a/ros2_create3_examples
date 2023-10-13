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

class OpticalFlowDemo(runner.OpenCvCode):
    def __init__(self, msg_queue):
        super().__init__(0, self.find_optic_ground, (np.pi / 4, 2), msg_queue)
        self.prev_gray = None

    def find_optic_ground(self, frame, cap, max_angle_min_magnitude):
        max_angle, min_magnitude = max_angle_min_magnitude
        if max_angle > np.pi:
            max_angle -= 2.0 * np.pi
        frame = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(frame)
        if self.prev_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            magnitude, angle = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            # Set the hue value of the mask image based on the flow direction
            mask[..., 0] = angle * 180 / np.pi / 2

            # Set the value of the mask image based on the flow magnitude
            mask[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)

            # Convert the mask image to BGR for visualization
            mask = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)
        self.prev_gray = gray
        return mask, []


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: remote_vision.RemoteBot(msg_queue, f'/{sys.argv[1]}'), OpticalFlowDemo(msg_queue))
