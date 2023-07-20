import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector, WheelStatus, WheelTicks
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from action_demo import RotateActionClient


class BumpTurnBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_turn_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.bump = None
        self.last_wheel_status = None
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bump is None:
            print("open")
            self.bump = runner.find_bump_from(msg.detections)
            if self.bump is None:
                self.publisher.publish(runner.straight_twist(0.5))
        elif self.wheels_stopped():
            goal = math.pi / 2
            if 'left' in self.bump:
                goal *= -1
            print("Starting turn", goal)
            self.rotator.send_goal(goal)
            rclpy.spin_once(self.rotator)
        else:
            print("waiting on wheels")

    def turn_finished_callback(self, future):
        self.bump = None
        print("finished with turn")

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg


if __name__ == '__main__':
    runner.run_single_node(lambda: BumpTurnBot(f'/{sys.argv[1]}'))
