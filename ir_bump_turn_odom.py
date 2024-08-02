import sys
import runner
import rclpy

import ir_turn, bump_turn_odom


class IrBumpTurnBot(runner.HdxNode):
    def __init__(self, namespace, ir_limit):
        super().__init__('ir_bump_turn_bot', namespace)
        self.add_child_node(ir_turn.IrTurnNode(namespace, ir_limit))
        self.add_child_node(bump_turn_odom.BumpTurnOdomNode(namespace))
        self.create_timer(0.10, self.timer_callback)

    def is_turning(self):
        return self['BumpTurnOdomNode'].is_turning() or self['IrTurnNode'].is_turning()

    def timer_callback(self):
        self.record_first_callback()
        if not self.is_turning():
            self.publish_twist(runner.straight_twist(0.5))


if __name__ == '__main__':
    rclpy.init()
    bot = IrBumpTurnBot(f'/{sys.argv[1]}', 50 if len(sys.argv) < 3 else int(sys.argv[2]))
    runner.run_recursive_node(bot)
