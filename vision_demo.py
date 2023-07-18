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
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)

        timer_period = 0.10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.img_queue = img_queue

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
            
    def ir_callback(self, msg):
        print('irs', [reading.value for reading in msg.readings])

    def bump_callback(self, msg):
        self.record_first_callback()
        for detected in msg.detections:
            if detected.header.frame_id != 'base_link':
                print(detected.header.frame_id, runner.BUMP_HEADINGS[detected.header.frame_id])



if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: VisionBot(msg_queue, f'/{sys.argv[1]}'), runner.OpenCvCode(0, lambda f, c, a: (f, None), None, msg_queue))
