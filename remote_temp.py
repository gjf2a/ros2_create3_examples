import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import InterfaceButtons
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from queue import Queue
import threading

from w1thermsensor import W1ThermSensor


key2twist = {
    'a': runner.turn_twist(0.5),
    'd': runner.turn_twist(-0.5),
    'w': runner.straight_twist(0.5),
    's': runner.straight_twist(-0.5)
}


class VisionBot(runner.HdxNode):
    def __init__(self, img_queue, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.odom = self.create_subscription(Odometry, f'{namespace}/odom', self.odom_callback, qos_profile_sensor_data)

        self.temperature = W1ThermSensor()

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.img_queue = img_queue

    def odom_callback(self, msg):
        self.record_first_callback()
        try:
            print(f"temperature: {self.temperature.get_temperature()}")
        except:
            print("Temperature not available yet")
        print(f"pose: {msg.pose.pose.position}")
        print(f"orientation: {msg.pose.pose.orientation}")
        

    def timer_callback(self):
        self.record_first_callback()
        if not self.img_queue.empty():
            msg = self.img_queue.get()
            if type(msg) == runner.CvKey:
                if msg.is_quit():
                    self.quit()
                elif msg.key in key2twist:
                    self.publisher.publish(key2twist[msg.key])

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: VisionBot(msg_queue, f'/{sys.argv[1]}'), runner.OpenCvCode(0, lambda f, c, a: (f, None), None, msg_queue))
