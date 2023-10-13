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

class RemoteBot(runner.HdxNode):
    def __init__(self, img_queue, namespace: str = "", velocity=0.3, show_ir=False, show_bump=False):
        super().__init__('remote_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 1)
        self.buttons = self.create_subscription(InterfaceButtons, namespace + '/interface_buttons', self.button_callback, qos_profile_sensor_data)
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)

        timer_period = 0.10 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.show_ir = show_ir
        self.show_bump = show_bump

        self.img_queue = img_queue

        self.last_key = None
        self.key2twist = {
            'w': runner.straight_twist(velocity),
            'a': runner.turn_twist(velocity),
            's': runner.straight_twist(-velocity),
            'd': runner.turn_twist(-velocity),
            'x': runner.straight_twist(0.0)
        }

    def timer_callback(self):
        self.record_first_callback()
        if not self.img_queue.empty():
            msg = self.img_queue.get()
            if type(msg) == runner.CvKey:
                if msg.is_quit():
                    self.quit()
                elif msg.key in self.key2twist:
                    self.last_key = msg.key
        if self.last_key is not None:
            self.publisher.publish(self.key2twist[self.last_key])

    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.quit()
            
    def ir_callback(self, msg):
        if self.show_ir:    
            print('irs', [reading.value for reading in msg.readings])

    def bump_callback(self, msg):
        self.record_first_callback()
        for detected in msg.detections:
            if self.show_bump and detected.header.frame_id != 'base_link':
                print(detected.header.frame_id, runner.BUMP_HEADINGS[detected.header.frame_id])



if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: RemoteBot(msg_queue, f'/{sys.argv[1]}'), runner.OpenCvCode(0, lambda f, c, a: (f, None), None, msg_queue))
