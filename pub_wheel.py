import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class WheelPublisher(Node):
    '''
    An example of publishing to a ROS2 topic. 
    '''

    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')

        self.publisher = self.create_publisher(
            Twist, namespace + '/cmd_vel', 10)

        timer_period = 5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.twist = Twist()
        self.twist.angular.z = 0.628

    def timer_callback(self):
        '''
        Purpose
        -------
        This function will be called every 5 seconds, right now it's 
        publishing the same colors every time. 
        Try and see if you can get the colors to change each time this function 
        is called!
        '''
        # NOTE Edit this line to change color
        self.publisher.publish(self.twist)


def main(args=None):
    print("starting main()")
    rclpy.init(args=args)

    print("constructor")
    led_publisher = WheelPublisher('/archangel')

    try:
        print("starting publisher")
        rclpy.spin(led_publisher)
    except KeyboardInterrupt:
        print('\nCaught Keyboard Interrupt')
    finally:
        print("Done")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        led_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
