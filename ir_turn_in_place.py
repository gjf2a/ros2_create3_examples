import sys
import runner
import rclpy
from ir_turn import IrTurnNode


if __name__ == '__main__':
    rclpy.init()
    bot = IrTurnNode(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
