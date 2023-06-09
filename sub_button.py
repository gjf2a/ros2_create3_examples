# Copied from https://github.com/paccionesawyer/Create3_ROS2_Intro/blob/main/individual_examples/sub_battery.py
# For the button messages: https://github.com/iRobotEducation/irobot_create_msgs

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import InterfaceButtons


class ButtonSubscriber(Node):
    '''
    An example of subscribing to a ROS2 topic.
    A Node listening to the /battery_state topic.
    '''

    def __init__(self, namespace: str = ""):
        '''
        Purpose
        -------
        initialized by calling the Node constructor, naming our node 
        'battery_subscriber'
        '''
        super().__init__('battery_subscriber')
        self.subscription = self.create_subscription(InterfaceButtons,
            namespace + '/interface_buttons', self.listener_callback,
            qos_profile_sensor_data)

    def listener_callback(self, msg: InterfaceButtons):
        '''
        Purpose
        -------
        Whenever our subscriber (listener) get's a message this function is 
        'called back' to and ran.
        '''
        self.get_logger().info('I heard: "%s"' % msg)
        self.printButton(msg)

    def printButton(self, msg):
        print(f"printButton(): {msg.button_1.is_pressed} {msg.button_2.is_pressed} {msg.button_power.is_pressed}")
        #print(dir(msg.button_1))
        #print(dir(msg.button_2))
        #print(dir(msg.button_power))


def main(args=None):
    rclpy.init(args=args)

    battery_subscriber = ButtonSubscriber("/archangel")
    try:
        rclpy.spin(battery_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print("Done")
        battery_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
