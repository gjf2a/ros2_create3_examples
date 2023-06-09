# Adapted from https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/sub_bumper_pub_LED.py
# Events and threads: see https://superfastpython.com/thread-event-object-in-python/

import runner
import time
import threading
import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds


class ColorPalette():
    """ Helper Class to define frequently used colors"""

    def __init__(self):
        self.red = LedColor(red=255, green=0, blue=0)
        self.green = LedColor(red=0, green=255, blue=0)
        self.blue = LedColor(red=0, green=0, blue=255)
        self.yellow = LedColor(red=255, green=255, blue=0)
        self.pink = LedColor(red=255, green=0, blue=255)
        self.cyan = LedColor(red=0, green=255, blue=255)
        self.purple = LedColor(red=127, green=0, blue=255)
        self.white = LedColor(red=255, green=255, blue=255)
        self.grey = LedColor(red=189, green=189, blue=189)
        self.tufts_blue = LedColor(red=98, green=166, blue=10)
        self.tufts_brown = LedColor(red=94, green=75, blue=60)


class BumperLightChange(runner.HdxNode):

    def __init__(self, namespace: str = ''):
        super().__init__('bumper_light_change')
        self.cp = ColorPalette()

        self.lights_publisher = self.create_publisher(
            LightringLeds, f'{namespace}/cmd_lightring', 10)

        self.subscription = self.create_subscription(
            HazardDetectionVector, f'{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data)

        self.lightring = LightringLeds()
        self.lightring.override_system = True
        self.first_bump_time = None
        self.done_waiting = False

    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. It then changes color based on that message.
        '''
        if not self.lightring.override_system:
            return
        if self.elapsed_time() > 40 and not self.done_waiting:
            print("Try the bumpers now")
            self.done_waiting = True

        self.record_first_callback()
            
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                if self.first_bump_time is None:
                    self.first_bump_time = self.elapsed_time()
                    print(f"first detected bump at: {self.first_bump_time}")
                print(f"{det} at {self.elapsed_time()}")
                if det == "bump_right":
                    light_list = self.make_uniform_light(self.cp.blue)
                elif det == "bump_left":
                    light_list = self.make_uniform_light(self.cp.red)
                elif det == "bump_front_left":
                    light_list = self.make_uniform_light(self.cp.pink)
                elif det == "bump_front_right":
                    light_list = self.make_uniform_light(self.cp.cyan)
                elif det == "bump_front_center":
                    light_list = self.make_uniform_light(self.cp.purple)

                current_time = self.get_clock().now()

                self.lightring.header.stamp = current_time.to_msg()
                self.lightring.leds = light_list
                
                self.lights_publisher.publish(self.lightring)
            # self.get_logger().info('I heard: "%s"' % msg)

    def make_uniform_light(self, color):
        return [color] * 6

    def reset(self):
        print("Inside reset")
        self.lightring.override_system = False
        self.lightring.leds = self.make_uniform_light(self.cp.white)
        self.lights_publisher.publish(self.lightring)


if __name__ == '__main__':
    runner.run_single_node(lambda: BumperLightChange('/archangel'))
