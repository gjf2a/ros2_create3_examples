import sys
import runner
import rclpy

from irobot_create_msgs.msg import HazardDetectionVector
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class BumpTurnBot(runner.WheelMonitorNode):
    def __init__(self, namespace: str = ""):
        super().__init__('simple_bump_turn', namespace)
        self.subscribe_hazard(self.bump_callback)

        self.bump = None
        self.create_timer(0.10, self.timer_callback)

    def bump_clear(self):
        return self.bump is None

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bump is None:
            self.bump = runner.find_bump_from(msg.detections)
            if self.bump:
                print(f"Detected {self.bump}")

    def timer_callback(self):
        self.record_first_callback()
        if self.bump_clear():
            self.publish_twist(runner.straight_twist(0.5))
        elif self.wheels_stopped():
            turn_amount = 1.0
            if 'left' in self.bump:
                turn_amount *= -1.0
            self.publish_twist(runner.turn_twist(turn_amount))
            self.bump = None

    def add_self_recursive(self, executor):
        executor.add_node(self)


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)

