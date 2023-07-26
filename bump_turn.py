import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector, WheelStatus, WheelTicks
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from action_demo import RotateActionClient


class BumpTurnNode(runner.HdxNode):
    def __init__(self, namespace: str = "", avoid_angle=math.pi / 2):
        super().__init__('bump_turn_node')
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.avoid_angle = avoid_angle
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)
        self.turning = False
        self.bump = None
        self.started = False

    def has_started(self):
        return self.started
        
    def bump_clear(self):
        return self.bump is None

    def is_turning(self):
        return self.turning

    def bump_callback(self, msg):
        self.record_first_callback()
        self.started = True
        if self.bump is None and not self.turning:
            self.bump = runner.find_bump_from(msg.detections)
            
    def start_turn(self):
        goal = self.avoid_angle
        if 'left' in self.bump:
            goal *= -1
        self.rotator.send_goal(goal)
        self.turning = True

    def turn_finished_callback(self, future):
        self.bump = None
        self.turning = False


class BumpTurnBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_turn_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.last_wheel_status = None
        self.bump_node = BumpTurnNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg

    def timer_callback(self):
        self.record_first_callback()
        if self.bump_node.has_started() and not self.bump_node.is_turning():
            if self.bump_node.bump_clear():
                self.publisher.publish(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.bump_node.start_turn()


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnBot(f'/{sys.argv[1]}')
    runner.run_multiple_nodes(bot, bot.bump_node, bot.bump_node.rotator)
