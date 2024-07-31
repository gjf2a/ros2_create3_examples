import runner

import sys

import rclpy
from geometry_msgs.msg import Twist


class TwistDemoBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('wheel_publisher')
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # From perplexity.ai, to turn 45 degrees allegedly
        # https://www.perplexity.ai/search/63aa4772-955b-497c-ba05-57a8513bf4cf?s=c
        self.msg = Twist()
        self.msg.linear.x = 0.1  # set linear velocity to 0.1 m/s
        self.msg.angular.z = 0.785  # set angular velocity to 0.785 rad/s (45 degrees/s)

        self.straight = Twist()
        self.straight.linear.x = 0.1
        self.straight.angular.z = 0.0

        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.stop.angular.z = 0.0

        self.actions = [self.msg, self.straight, self.stop]
        self.action = 0

    def timer_callback(self):
        self.record_first_callback()
        self.publisher.publish(self.actions[self.action])
        self.action = (self.action + 1) % len(self.actions)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Supply the robot name as an arg")
    else:
        print(f"Starting up {sys.argv[1]}...")
        runner.run_single_node(lambda: TwistDemoBot(f'/{sys.argv[1]}'))
