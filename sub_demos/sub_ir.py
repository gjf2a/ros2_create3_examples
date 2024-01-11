import sys
import runner
import rclpy
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data


class IrBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('ir_subscriber')
        self.irs = self.create_subscription(IrIntensityVector, f"{namespace}/ir_intensity", self.ir_callback, qos_profile_sensor_data)

    def ir_callback(self, msg):
        self.record_first_callback()
        print('irs', [reading.value for reading in msg.readings])
        print(self.get_clock().now(), self.get_clock().now().nanoseconds)


if __name__ == '__main__':
    runner.run_single_node(lambda: IrBot(f'/{sys.argv[1]}'))
