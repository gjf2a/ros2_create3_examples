import sys
import math
import runner
import rclpy

from runner import RotateActionClient


class BumpTurnNode(runner.HdxNode):
    def __init__(self, namespace: str = "", avoid_angle=math.pi/2,
                 avoid_distribution_width=math.pi/4, avoid_random_vars=2):
        super().__init__('bump_turn_node', namespace)
        self.subscribe_hazard(self.bump_callback)
        self.avoid_angle = avoid_angle
        self.avoid_distribution_width = avoid_distribution_width
        self.avoid_random_vars = avoid_random_vars
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
        goal = runner.discretish_norm(self.avoid_angle, self.avoid_distribution_width, self.avoid_random_vars)
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
        self.bump_node = BumpTurnNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if not self.bump_node.is_turning():
            if self.bump_node.bump_clear():
                self.publish_twist(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.bump_node.start_turn()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.bump_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
