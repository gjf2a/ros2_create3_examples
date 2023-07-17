import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data

from morph_contour_demo import Timer, morph_contour_loop

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
        timer_period = 0.25 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Required for "Type Anything to Quit" from runner.py"

        self.img_queue = img_queue

    def timer_callback(self):
        self.record_first_callback()
        if not self.img_queue.empty():
            best = self.img_queue.get()
            if best == "QUIT":
                self.quit()
            else:
                x_center = sum(p[0][0] for p in best) / len(best)
                fuzzy_center = fuzzify_falling(x_center, 0, 640)
                msg = Twist()
                msg.linear.x = 0.1  
                msg.angular.z = defuzzify(fuzzy_center, -0.785, 0.785)
                print(fuzzy_center, msg.angular.z)
                self.publisher.publish(msg)

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
    queue = Queue()
    vt = threading.Thread(target=morph_contour_loop, args=(0, 9, 20, queue))
    vt.start()
    while queue.empty():
        pass
    print(f"Starting up {sys.argv[1]}...")
    runner.run_single_node(lambda: VisionBot(queue, f'/{sys.argv[1]}'))
