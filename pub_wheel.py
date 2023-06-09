# Based on https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/pub_LED.py

import runner
import sys
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class WheelPublisher(Node):
    '''
    An example of publishing to a ROS2 topic. 
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')

        self.start = time.time()

        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)

        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist = Twist()
        self.twist.angular.z = 0.628
        self.first_callback_time = None

    def elapsed(self):
        return time.time() - self.start

    def timer_callback(self):
        if self.first_callback_time is None:
            self.first_callback_time = self.elapsed()
        print(f"callback: {self.elapsed()}")
        self.publisher.publish(self.twist)


def main(args=None):
    print("starting main()")
    rclpy.init(args=args)

    print("constructor")
    wheel_publisher = WheelPublisher('/archangel')

    try:
        print(f"starting publisher at {wheel_publisher.elapsed()}")
        rclpy.spin(wheel_publisher)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        wheel_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    runner.run_single_node(lambda: WheelPublisher('/archangel'))
