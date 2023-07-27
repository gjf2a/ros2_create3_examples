import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector
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
            if self.bump:
                print(f"Detected {self.bump}")
            
    def start_turn(self):
        goal = self.avoid_angle
        if 'left' in self.bump:
            goal *= -1
        self.rotator.send_goal(goal)
        self.turning = True
        print(f"Sending goal: {goal}")

    def turn_finished_callback(self, future):
        print("BumpTurnNode: Turn finished")
        self.bump = None
        self.turning = False

    def add_self_recursive(self, executor):
        executor.add_node(self)
        executor.add_node(self.rotator)


class BumpTurnBot(runner.WheelMonitorNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_turn_bot', namespace)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.bump_node = BumpTurnNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if self.bump_node.has_started() and not self.bump_node.is_turning():
            if self.bump_node.bump_clear():
                self.publisher.publish(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.bump_node.start_turn()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.bump_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
