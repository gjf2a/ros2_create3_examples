import rclpy
from qlearning import QBot, QParameters, QTable
import runner
import math
import sys

from irobot_create_msgs.msg import WheelStatus, HazardDetectionVector, IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

class QDemoNode(runner.HdxNode):
    def __init__(self, namespace, max_ir=50):
        super().__init__('q_demo_node')
        self.max_ir = max_ir
        self.wheel_status = self.create_subscription(WheelStatus, f'{namespace}/wheel_status', self.wheel_status_callback, qos_profile_sensor_data)
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.action_twists = [runner.straight_twist(0.5), runner.turn_twist(math.pi / 4)]
        self.last_action = None
        self.pending_action = None
        self.state = None
        self.reward = None

    def num_actions(self):
        return len(self.action_twists)

    def num_states(self):
        return 3

    def action_in_progress(self):
        return self.pending_action is not None

    def read_state(self):
        state = self.state
        self.state = None
        if state == 2:
            self.reward = -100
        elif self.last_action == 0:
            self.reward = 1
        else:
            self.reward = 0
        return state

    def read_reward(self):
        return self.reward

    def act(self, action_num):
        if self.pending_action is None:
            action = self.action_twists[action_num]
            if action_num == self.last_action:
                self.publisher.publish(action)
            else:
                self.last_action = action_num
                self.pending_action = action

    def wheel_status_callback(self, msg):
        self.record_first_callback()
        if self.pending_action is not None and msg.current_ma_left == 0 and msg.current_ma_right == 0:
            self.publisher.publish(self.pending_action)
            self.pending_action = None

    def bump_callback(self, msg):
        bump = runner.find_bump_from(msg.detections)
        if bump is not None:
            self.state = 2

    def ir_callback(self, msg):
        self.record_first_callback()
        if self.state != 2:
            ir_values = [reading.value for reading in msg.readings]
            if max(ir_values) > self.max_ir:
                self.state = 1
            else:
                self.state = 0

    def add_self_recursive(self, executor):
        executor.add_node(self)
            

if __name__ == '__main__':
    rclpy.init()
    namespace = f'/{sys.argv[1]}' if len(sys.argv) >= 2 else ''
    params = QParameters()
    demo_node = QDemoNode(namespace)
    main_node = QBot(demo_node, params)
    runner.run_recursive_node(main_node)
    
