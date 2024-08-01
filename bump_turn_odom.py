import math, sys
import runner
import rclpy
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector


HAZARD_TURNS = {
    'left': -math.pi / 4,
    'front_left': -math.pi / 2,
    'front_center': math.pi / 2,
    'front_right': math.pi / 2,
    'right': math.pi / 4
}


class BumpTurnOdomNode(runner.OdomMonitorNode):
    def __init__(self, namespace: str = "", avoid_random_vars=2):
        super().__init__('bump_turn_odom', namespace)
        self.subscribe_hazard(self.hazard_callback)
        self.subscribe_odom(self.odom_callback)
        self.avoid_random_vars = avoid_random_vars
        self.last_pose = None
        self.heading_goal = None

    def is_turning(self) -> bool:
        return self.heading_goal is not None

    def odom_callback(self, msg: Odometry):
        super().odom_callback(msg)
        if self.is_turning():
            if abs(runner.angle_diff(self.heading_goal, self.last_heading())) < math.pi/32:
                self.heading_goal = None
            else:
                self.publish_twist(runner.turn_twist_towards(math.pi/4, self.last_heading(), self.heading_goal))

    def hazard_callback(self, msg: HazardDetectionVector):
        hazard = runner.find_hazard_from(msg.detections)
        if hazard is not None and self.has_position():
            suffix = runner.hazard_id_suffix(hazard)
            angle_center = HAZARD_TURNS[suffix]
            goal = runner.discretish_norm(angle_center, angle_center / 2, self.avoid_random_vars)
            self.heading_goal = self.last_heading() + goal


class BumpTurnOdomBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_turn_bot', namespace)
        self.add_child_node(BumpTurnOdomNode(namespace))
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if not self['BumpTurnOdomNode'].is_turning():
            self.publish_twist(runner.straight_twist(0.5))


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnOdomBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
