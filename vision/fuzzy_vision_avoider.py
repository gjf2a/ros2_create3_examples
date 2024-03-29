import sys
import runner
import time
import rclpy
import cv2
import math

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, WheelStatus
from rclpy.qos import qos_profile_sensor_data
from runner import RotateActionClient

from morph_contour_demo import Timer, find_contours, find_close_contour, find_contour_clusters, best_contour_cluster

from queue import Queue
import threading

from fuzzy import *


class VisionBot(runner.HdxNode):
    def __init__(self, img_queue, namespace: str = "", use_ir=False):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        timer_period = 0.10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.img_queue = img_queue
        self.last_wheel_status = None
        self.avoid_direction = None
        self.actively_turning = False
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)

        self.use_ir = use_ir

        self.last_x_y = None

    def use_vision(self):
        return self.avoid_direction is None

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg

    def timer_callback(self):
        self.record_first_callback()
        if not self.img_queue.empty():
            best = self.img_queue.get()
            if best is None:
                print("Contour finder failed")
            elif best == "QUIT":
                self.quit()
            elif type(best) == runner.CvKey:
                print("Typed", best)
            elif self.use_vision():
                x_center = sum(p[0][0] for p in best) / len(best)
                y_center = sum(p[0][1] for p in best) / len(best)
                fuzzy_center = fuzzify_falling(x_center, 0, 640)
                msg = Twist()
                msg.linear.x = 0.1  
                msg.angular.z = defuzzify(fuzzy_center, -math.pi/4, math.pi/4)
                print(f"best: ({x_center}, {y_center}) {fuzzy_center} {msg.angular.z}")
                self.publisher.publish(msg)

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            
    def ir_callback(self, msg):
        ir_values = [reading.value for reading in msg.readings]
        if self.use_ir and self.use_vision() and max(ir_values) > 50:
            mid = len(ir_values) // 2
            self.avoid_direction = math.pi / 4
            if sum(ir_values[:mid]) < sum(ir_values[-mid:]):
                self.avoid_direction *= -1.0
            print("IR detects trouble - avoiding", ir_values, self.avoid_direction)

    def bump_callback(self, msg):
        if self.use_vision():
            bump = runner.find_bump_from(msg.detections)
            if bump is not None:
                self.avoid_direction = math.pi / 4
                if 'left' in bump:
                    self.avoid_direction *= -1
        elif self.wheels_stopped() and not self.actively_turning:
            self.actively_turning = True
            print("Starting turn")
            self.rotator.send_goal(self.avoid_direction)

    def turn_finished_callback(self, future):
        self.avoid_direction = None
        self.actively_turning = False
        print("Finished with turn", future.result().result)


def find_floor_contour(frame, cap, kernel_midwidth):
    kernel, min_space_width = kernel_midwidth
    kernel_size = (kernel, kernel)

    frame = cv2.resize(frame, (640, 480))
    contours, hierarchy = find_contours(frame, kernel_size)
    close_contour = find_close_contour(contours, cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    if close_contour is None:
        print("contour finding failure")
        best = None
    else:
        clusters = find_contour_clusters(close_contour)
        best = best_contour_cluster(clusters, min_space_width)

        cv2.drawContours(frame, close_contour, -1, (255, 0, 0), 3)
        cv2.drawContours(frame, best, -1, (0, 0, 255), 3)

    return frame, best



class FloorContour(runner.OpenCvCode):
    def __init__(self, msg_queue):
        super().__init__(0, find_floor_contour, (9, 20), msg_queue)


if __name__ == '__main__':
    rclpy.init()
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    bot = VisionBot(msg_queue, f'/{sys.argv[1]}', use_ir='-ir' in sys.argv)
    runner.run_vision_multiple_nodes(FloorContour(msg_queue), bot, bot.rotator)
