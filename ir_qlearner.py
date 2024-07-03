import rclpy
from qlearning import QBot, QParameters, QTable, QNodeTemplate
import runner
import math
import sys
import numpy as np

from irobot_create_msgs.msg import HazardDetectionVector, IrIntensityVector
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist

class QDemoNode(QNodeTemplate):
    def __init__(self, namespace, max_ir=50):
        super().__init__('q_demo_node', namespace, runner.straight_twist(0.5), runner.turn_twist(math.pi / 4), runner.turn_twist(-math.pi / 4))
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)
        self.irVectors = [np.array([0,0,0,0,0,0,0]).np.array([1,1,1,1,1,1,1]),np.array([2,2,2,2,2,2,2]),np.array([3,3,3,3,3,3,3]),np.array([4,4,4,4,4,4,4]),np.array([5,5,5,5,5,5,5]),np.array([6,6,6,6,6,6,6]),np.array([7,7,7,7,7,7,7,7]),np.array([8,8,8,8,8,8,8,8])]

    def num_states(self):
        return 3

    def set_reward(self, state):
        if state == 10:
            return -100
        elif self.last_action == 0:
            return 1
        else:
            return 0

    def bump_callback(self, msg):
        bump = runner.find_bump_from(msg.detections)
        if bump is not None:
            self.state = 9

    def ir_callback(self, msg):
        self.record_first_callback()
        if self.state != 9:
            values = []
            for reading in msg.readings:
                val = reading.value
                values.append(val)
            values = np.array(values)
            minDist = -1
            for i in range(len(self.irVectors))
                dist = np.linalg.norm(values,irVectors[i])
                if minDist == -1 or minDist > dist
                    minDist = dist
                    self.state = i

if __name__ == '__main__':
    rclpy.init()
    namespace = f'/{sys.argv[1]}' if len(sys.argv) >= 2 else ''
    params = QParameters()
    params.epsilon = 0.05
    demo_node = QDemoNode(namespace)
    main_node = QBot(demo_node, params)
    runner.run_recursive_node(main_node)
    main_node.print_status()
