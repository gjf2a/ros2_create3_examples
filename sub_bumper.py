import sys
import runner
import rclpy
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data


class BumperBot(runner.HdxNode):
    def __init__(self, namespace: str = ""):
        super().__init__('bump_subscriber')
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)

    def bump_callback(self, msg):
        self.record_first_callback()
        for detected in msg.detections:
            if detected.header.frame_id != 'base_link':
                print(detected.header.frame_id, runner.BUMP_HEADINGS[detected.header.frame_id])


if __name__ == '__main__':
    runner.run_single_node(lambda: BumperBot(f'/{sys.argv[1]}'))
