import bump_turn
import rclpy
import runner
import sys


class Overridden(bump_turn.BumpTurnBot):
    def __init__(self, namespace):
        super().__init__(namespace)
        print("init!!!")

    def timer_callback(self):
        super().timer_callback()
        print("Overridden!")


if __name__ == '__main__':
    rclpy.init()
    bot = Overridden(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
