import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

from runner import RotateActionClient
import time


class IrBumpTurnNode(runner.HdxNode):
    def __init__(self, namespace: str = "", ir_too_close=50, turn_velocity=math.pi/4):
        super().__init__('ir_turn_node')
        self.ir_too_close = ir_too_close
        self.turn_velocity = turn_velocity
        self.publisher = self.create_publisher(Twist, f'{namespace}/cmd_vel', 10)
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.avoid_direction = None
        self.turn_requested = False
        self.turn_started = False

    def ir_clear(self):
        return self.avoid_direction is None

    def is_turning(self):
        return self.turn_requested

    def turn_pending(self):
        return self.avoid_direction is not None and not self.turn_started

    def bump_callback(self, msg):
        self.record_first_callback()
        bump = runner.find_bump_from(msg.detections)
        if bump is not None and self.avoid_direction is None:
            self.avoid_direction = self.turn_velocity
            if 'left' in bump:
                self.avoid_direction *= -1.0

    def ir_callback(self, msg):
        self.record_first_callback()
        ir_values = [reading.value for reading in msg.readings]
        max_ir = max(ir_values)
        if self.turn_pending() or max_ir > self.ir_too_close:
            if self.turn_requested:
                self.publisher.publish(runner.turn_twist(self.avoid_direction))
                self.turn_started = True
            else:
                mid = len(ir_values) // 2
                self.avoid_direction = self.turn_velocity
                if sum(ir_values[:mid]) < sum(ir_values[-mid:]):
                    self.avoid_direction *= -1.0
        elif not self.turn_pending():
            self.avoid_direction = None
            self.turn_requested = False
            self.turn_started = False

    def request_turn_until_clear(self):
        self.turn_requested = True

    def add_self_recursive(self, executor):
        executor.add_node(self)


class IrBumpTurnBot(runner.WheelMonitorNode):
    def __init__(self, namespace: str = "", ir_limit=50):
        super().__init__('ir_turn_bot', namespace)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.ir_node = IrBumpTurnNode(namespace, ir_limit)
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if not self.ir_node.is_turning():
            if self.ir_node.ir_clear():
                self.publisher.publish(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.ir_node.request_turn_until_clear()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.ir_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    bot = IrBumpTurnBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
