# Based on https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/pub_LED.py

import runner
import sys
import time
import rclpy

from geometry_msgs.msg import Twist


class WheelPublisher(runner.HdxNode):
    '''
    An example of publishing to a ROS2 topic. 
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')

        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist = Twist()
        #self.twist.angular.z = 0.628
        self.twist.linear.x = 1.0

    def timer_callback(self):
        self.record_first_callback()
        print(f"callback: {self.elapsed_time()}")
        self.publisher.publish(self.twist)


if __name__ == '__main__':
    runner.run_single_node(lambda: WheelPublisher('/archangel'))
