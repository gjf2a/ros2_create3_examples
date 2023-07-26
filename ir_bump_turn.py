import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import IrIntensityVector, WheelStatus, WheelTicks
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from action_demo import RotateActionClient
from ir_turn import IrTurnNode
from bump_turn import BumpTurnNode


class IrBumpTurnBot(runner.HdxNode):
    def __init__(self, namespace: str = "", ir_limit=50):
        super().__init__('ir_bump_turn_bot')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.last_wheel_status = None
        self.ir_node = IrTurnNode(namespace, ir_limit)
        self.bump_node = BumpTurnNode(namespace)
        self.create_timer(0.10, self.timer_callback)

# Continue work from here down

    def timer_callback(self):
        self.record_first_callback()
        if not self.ir_node.is_turning():
            if self.ir_node.ir_clear():
                self.publisher.publish(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.ir_node.start_turn_until_clear()

    def wheels_stopped(self):
        return self.last_wheel_status is not None and self.last_wheel_status.current_ma_left == 0 and self.last_wheel_status.current_ma_right == 0

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        self.last_wheel_status = msg


if __name__ == '__main__':
    rclpy.init()
    bot = IrBumpTurnBot(f'/{sys.argv[1]}')
    runner.run_multiple_nodes(bot, bot.ir_node)