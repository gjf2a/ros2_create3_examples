import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data

from morph_contour_demo import Timer, contour_inner_loop
import cv2


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
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        #self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_callback, qos_profile_sensor_data)
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Required for "Type Anything to Quit" from runner.py"
        self.forward = Twist()
        self.forward.linear.x = 0.5
        self.turn_left = Twist()
        self.turn_left.angular.z = 1.0
        self.turn_right = Twist()
        self.turn_right.angular.z = -1.0

        # OpenCV stuff
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Can't open camera - exiting")
            sys.exit(1)
        self.kernel_size = (9, 9)

    def timer_callback(self):
        self.record_first_callback()
        frame, contours, close_contour, best = contour_inner_loop(self.cap, self.kernel_size, 20)
        x_center = sum(p[0][0] for p in best) / len(best)
        fuzzy_left = fuzzify_rising(x_center, 0, 320)
        fuzzy_right = fuzzify_falling(x_center, 320, 640)
        print(fuzzy_left, fuzzy_right)

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            
    def ir_callback(self, msg):
        print(f"IR callback at {self.elapsed_time()}")
        for reading in msg.readings:
            det = reading.header.frame_id
            val = reading.value
            if det != "base_link":
                self.ir_check(det, val)
        if self.ir_clear_count == 7:
           self.publisher.publish(self.forward)
        self.ir_clear_count = 0

    def ir_check(self, sensor: str = "", val: int = 0):
        if val > 100:
            if sensor.endswith("right"):
                self.publisher.publish(self.turn_left)
            elif sensor.endswith("left"):
                self.publisher.publish(self.turn_right)
            print(sensor)
            self.irs.add(sensor)
            print(self.irs)
        else:
            self.ir_clear_count += 1

if __name__ == '__main__':
    print(f"Starting up {sys.argv[1]}...")
    runner.run_single_node(lambda: VisionBot(f'/{sys.argv[1]}'))
