import sys
import runner
import rclpy

import ir_turn, bump_turn_odom


class IrBumpTurnBot(runner.WheelMonitorNode):
    def __init__(self, namespace, ir_limit):
        super().__init__('ir_bump_turn_bot', namespace)
        self.ir_node = ir_turn.IrTurnNode(namespace, ir_limit)
        self.bump_node = bump_turn_odom.BumpTurnOdomNode(namespace)
        self.create_timer(0.10, self.timer_callback)

    def is_turning(self):
        return self.bump_node.is_turning() or self.ir_node.is_turning()

    def timer_callback(self):
        self.record_first_callback()
        if not self.is_turning():
            if self.ir_node.ir_clear():
                self.publish_twist(runner.straight_twist(0.5))
            elif self.wheels_stopped():
                self.ir_node.start_turn_until_clear()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.bump_node.add_self_recursive(executor)
        self.ir_node.add_self_recursive(executor)


if __name__ == '__main__':
    rclpy.init()
    ir_limit = 50 if len(sys.argv) < 3 else int(sys.argv[2])

    bot = IrBumpTurnBot(f'/{sys.argv[1]}', ir_limit)
    runner.run_recursive_node(bot)