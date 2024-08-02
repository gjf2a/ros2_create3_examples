import sys, datetime
import runner
import trajectories
import rclpy

import ir_bump_turn_odom


class TrajectoryMapper(ir_bump_turn_odom.IrBumpTurnBot):
    def __init__(self, namespace, ir_limit):
        super().__init__(namespace, ir_limit)
        self.map = trajectories.TrajectoryMap()

    def timer_callback(self):
        super().timer_callback()
        x, y = self['BumpTurnOdomNode'].last_x_y()
        self.map.update(x, y)


if __name__ == '__main__':
    rclpy.init()
    ir_limit = 50 if len(sys.argv) < 3 else int(sys.argv[2])

    bot = TrajectoryMapper(f'/{sys.argv[1]}', ir_limit)
    runner.run_recursive_node(bot)
    with open(f"trajectory_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'w') as file:
        file.write(f"{bot.map}\n")
