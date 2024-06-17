import sys
import math
import runner
import rclpy

from runner import RotateActionClient, DriveDistanceClient


class BumpTurnNode(runner.HdxNode):
    def __init__(self, namespace: str = "", avoid_angle=math.pi / 2, drive_dist=0.5):
        super().__init__('bump_turn_node', namespace)
        self.subscribe_hazard(self.bump_callback)
        self.avoid_angle = avoid_angle
        self.drive_dist = drive_dist
        self.rotator = RotateActionClient(self.turn_finished_callback, namespace)
        self.driver = DriveDistanceClient(self.drive_finished_callback, namespace)
        self.driving = False
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
        if self.driving:
            self.driver.cancel()
        goal = self.avoid_angle
        if 'left' in self.bump:
            goal *= -1
        self.rotator.send_goal(goal)
        self.turning = True
        print(f"Sending turn goal: {goal}")

    def start_drive(self):
        if not self.driving:
            self.driver.send_goal(self.drive_dist)
            self.driving = True
            print(f"Sending drive goal: {self.drive_dist}")

    def turn_finished_callback(self, future):
        print("BumpTurnNode: Turn finished")
        self.bump = None
        self.turning = False

    def drive_finished_callback(self, future):
        print("BumpTurnNode: Drive finished")
        self.driving = False

    def add_self_recursive(self, executor):
        executor.add_node(self)
        executor.add_node(self.rotator)
        executor.add_node(self.driver)


class BumpTurnBot(runner.WheelMonitorNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_turn_bot', namespace)
        self.bump_node = BumpTurnNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def timer_callback(self):
        self.record_first_callback()
        if self.bump_node.has_started() and not self.bump_node.is_turning():
            if self.bump_node.bump_clear():
                self.bump_node.start_drive()
            elif self.wheels_stopped():
                self.bump_node.start_turn()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.bump_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    bot = BumpTurnBot(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
