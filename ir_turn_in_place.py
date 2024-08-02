import sys
import math
import runner
import rclpy
from irobot_create_msgs.msg import IrIntensityVector
from ir_turn import IrTurnNode


if __name__ == '__main__':
    rclpy.init()
    bot = IrTurnNode(f'/{sys.argv[1]}')
    runner.run_recursive_node(bot)
