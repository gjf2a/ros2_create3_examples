import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist


class BumperBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_subscriber')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)

    def bump_callback(self, msg):
        self.record_first_callback()
        bump = runner.find_bump_from(msg.detections)
        if bump is None:
            self.publisher.publish(runner.straight_twist(0.5))
        else:
            print(bump)
            self.publisher.publish(Twist())
            #if 'left' in bump:
            #    self.publisher.publish(runner.turn_twist(math.pi/4))
            #else:
            #    self.publisher.publish(runner.turn_twist(-math.pi/4))


if __name__ == '__main__':
    runner.run_single_node(lambda: BumperBot(f'/{sys.argv[1]}'))
