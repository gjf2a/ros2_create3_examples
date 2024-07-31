import math, sys
import runner
import rclpy
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector


class BumpTurnOdom(runner.OdomMonitorNode):
    def __init__(self, namespace: str = "", avoid_angle=math.pi/2,
                 avoid_distribution_width=math.pi/4, avoid_random_vars=2):
        super().__init__('bump_turn_odom', namespace)
        self.subscribe_hazard(self.hazard_callback)
        self.subscribe_odom(self.odom_callback)
        self.avoid_angle = avoid_angle
        self.avoid_distribution_width = avoid_distribution_width
        self.avoid_random_vars = avoid_random_vars
        self.last_pose = None
        self.heading_goal = None

    def odom_callback(self, msg: Odometry):
        super().odom_callback(msg)
        if self.heading_goal is None:
            self.publish_twist(runner.straight_twist(0.5))
        elif abs(runner.angle_diff(self.heading_goal, self.last_heading())) < math.pi/32:
            self.heading_goal = None
        else:
            self.publish_twist(runner.turn_twist_towards(math.pi/4, self.last_heading(), self.heading_goal))

    def hazard_callback(self, msg: HazardDetectionVector):
        bump = runner.find_bump_from(msg.detections)
        if bump is not None and self.has_position():
            goal = runner.discretish_norm(self.avoid_angle, self.avoid_distribution_width, self.avoid_random_vars)
            if 'left' in bump:
                goal *= -1
            self.heading_goal = self.last_heading() + goal

    def add_self_recursive(self, executor):
        executor.add_node(self)


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnOdom(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
