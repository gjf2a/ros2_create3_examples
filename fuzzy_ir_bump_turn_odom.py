import sys
from typing import Tuple

import runner
import rclpy

import ir_turn, bump_turn_odom
from fuzzy import *


class IrBumpTurnBot(runner.HdxNode):
    def __init__(self, namespace, ir_limit):
        super().__init__('ir_bump_turn_bot', namespace)
        self.ir_turn = ir_turn.IrTurnNode(namespace, ir_limit)
        self.bump_turn = bump_turn_odom.BumpTurnOdomNode(namespace)
        self.add_child_nodes(self.ir_turn, self.bump_turn)
        self.create_timer(0.10, self.timer_callback)
        self.subscribe_ir(self.slow_down_callback)
        self.forward_factor = 1.0

    def last_x_y(self) -> Tuple[float, float]:
        return self.bump_turn.last_x_y()

    def is_turning(self):
        return self.bump_turn.is_turning() or self.ir_turn.is_turning()

    def timer_callback(self):
        self.record_first_callback()
        if self.bump_turn.is_turning():
            self.ir_turn.pause()
        else:
            self.ir_turn.resume()
            if self.ir_turn.ir_clear():
                self.publish_twist(runner.straight_twist(self.forward_factor * 0.5))

    def slow_down_callback(self, msg):
        ir_values = [reading.value for reading in msg.readings]
        max_ir = max(ir_values)
        self.forward_factor = max(0.2, fuzzify_falling(max_ir, 10, 50))


if __name__ == '__main__':
    rclpy.init()
    bot = IrBumpTurnBot(f'/{sys.argv[1]}', 50 if len(sys.argv) < 3 else int(sys.argv[2]))
    runner.run_recursive_node(bot)
