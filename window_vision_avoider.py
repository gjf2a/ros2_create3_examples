import runner
import sys
import time
import rclpy
import cv2
import math

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector, HazardDetectionVector, WheelStatus
from rclpy.qos import qos_profile_sensor_data
from action_demo import RotateActionClient

from morph_contour_demo import Timer, find_contours, find_close_contour, find_contour_clusters, best_contour_cluster

from queue import Queue
import threading


def fuzzify_rising(value, start, end):
    if value > end:
        return 1.0
    elif value < start:
        return 0.0
    else:
        return (value - start) / (end - start)


def fuzzify_falling(value, start, end):
    return f_not(fuzzify_rising(value, start, end))


def f_not(value):
    return 1.0 - value


def defuzzify(value, zero, one):
    if zero > one:
        return defuzzify(f_not(value), one, zero)
    else:
        return zero + value * (one - zero)


class VisionBot(runner.HdxNode):
    def __init__(self, img_queue, namespace: str = ""):
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
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)

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
            if best == "QUIT":
                self.quit()
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
        if max(ir_values) > 50:
            print("IR detects trouble - avoiding")
            mid = len(ir_values) // 2
            self.avoid_direction = math.pi / 4
            if sum(ir_values[:mid]) < sum(ir_values[-mid:]):
                self.avoid_direction *= -1.0

    def bump_callback(self, msg):
        if self.use_vision():
            bump = runner.find_bump_from(msg.detections)
            if bump is not None:
                self.avoid_direction = math.pi / 4
                if 'left' in bump:
                    self.avoid_direction *= -1
        elif self.wheels_stopped():
            print("Starting turn")
            self.rotator.send_goal(self.avoid_direction)
            rclpy.spin_once(self.rotator)

    def turn_finished_callback(self, future):
        self.avoid_direction = None
        print("Finished with turn")


def find_floor_contour(frame, cap, kernel_midwidth):
    kernel, min_space_width = kernel_midwidth
    kernel_size = (kernel, kernel)

    frame = cv2.resize(frame, (640, 480))
    contours, hierarchy = find_contours(frame, kernel_size)
    close_contour = find_close_contour(contours, cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    clusters = find_contour_clusters(close_contour)
    best = best_contour_cluster(clusters, min_space_width)

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
    cv2.drawContours(frame, close_contour, -1, (255, 0, 0), 3)
    cv2.drawContours(frame, best, -1, (0, 0, 255), 3)

    return frame, best



class FloorContour(runner.OpenCvCode):
    def __init__(self, msg_queue):
        super().__init__(0, find_floor_contour, (9, 20), msg_queue)


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: VisionBot(msg_queue, f'/{sys.argv[1]}'), FloorContour(msg_queue))
