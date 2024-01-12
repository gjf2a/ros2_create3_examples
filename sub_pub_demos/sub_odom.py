# Adapted from https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/sub_battery.py

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdometrySubscriber(Node):
    '''
    An example of subscribing to a ROS2 topic.
    A Node listening to the /odom topic.
    '''

    def __init__(self, namespace: str = ""):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'odometry_subscriber'
        '''
        super().__init__('odometry_subscriber')
        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg: Odometry):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''
        self.printOdometry(msg)

    def printOdometry(self, msg: Odometry):
        '''
        An example of how to get components of the msg returned from a topic.
        '''
        # We can get components of the message by using the '.' dot operator
        p = msg.pose.pose.position
        print(f"Position: ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")


def main(args=None):
    rclpy.init(args=args)

    odometry_subscriber = OdometrySubscriber("/archangel")
    try:
        rclpy.spin(odometry_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        odometry_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
