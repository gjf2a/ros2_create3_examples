import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector, WheelStatus, WheelTicks
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist


class BumpStopBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_subscriber')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.bumped = False
        self.last_wheel_status = None

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bumped:
            print(self.last_wheel_status.current_ma_left, self.last_wheel_status.current_ma_right)
        else:
            bump = runner.find_bump_from(msg.detections)
            if bump is None:
                self.publisher.publish(runner.straight_twist(0.5))
            else:
                print(bump)
                self.bumped = True

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg


if __name__ == '__main__':
    runner.run_single_node(lambda: BumpStopBot(f'/{sys.argv[1]}'))
