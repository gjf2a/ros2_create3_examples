import runner
import sys
import time
import rclpy
import cv2
import math

from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from action_demo import RotateActionClient

from morph_contour_demo import Timer, find_contours, find_close_contour, find_contour_clusters, best_contour_cluster

from queue import Queue
import threading

from ir_turn import IrTurnNode
from bump_turn import BumpTurnNode


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


class VisionBot(runner.WheelMonitorNode):
    def __init__(self, img_queue, namespace: str = "", ir_limit=50):
        super().__init__('vision_ir_bump_avoid_bot', namespace)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        timer_period = 0.10 # seconds
        self.create_timer(timer_period, self.timer_callback)

        self.img_queue = img_queue
        self.ir_node = IrTurnNode(namespace, ir_limit)
        self.bump_node = BumpTurnNode(namespace)

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.bump_node.add_self_recursive(executor)
        self.ir_node.add_self_recursive(executor)

    def use_vision(self):
        return self.bump_node.bump_clear() and self.ir_node.ir_clear()

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
            elif not self.bump_node.is_turning():
                if self.bump_node.bump_clear():
                    if not self.ir_node.is_turning():
                        if self.ir_node.ir_clear():
                            x_center = sum(p[0][0] for p in best) / len(best)
                            y_center = sum(p[0][1] for p in best) / len(best)
                            fuzzy_center = fuzzify_falling(x_center, 0, 640)
                            msg = Twist()
                            msg.linear.x = 0.1  
                            msg.angular.z = defuzzify(fuzzy_center, -math.pi/4, math.pi/4)
                            print(f"best: ({x_center}, {y_center}) {fuzzy_center} {msg.angular.z}")
                            self.publisher.publish(msg)
                        elif self.wheels_stopped():
                            self.ir_node.start_turn_until_clear()
                            print("Starting IR turn")
                elif self.wheels_stopped():
                    self.bump_node.start_turn()
                    print("Starting bump turn")


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
    bot = VisionBot(msg_queue, f'/{sys.argv[1]}')
    runner.run_recursive_vision_node(FloorContour(msg_queue), bot)
