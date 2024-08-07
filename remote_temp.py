# To reset the odometry:
#
# ros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose "{pose:{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

import runner
import sys
import time
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from queue import Queue

from w1thermsensor import W1ThermSensor
from w1thermsensor.units import Unit

import remote_vision


class TemperatureBot(remote_vision.RemoteBot):
    def __init__(self, img_queue, namespace: str = "", temp_file: str = "temps.txt"):
        super().__init__(img_queue, namespace)
        self.odom = self.create_subscription(Odometry, f'{namespace}/odom', self.odom_callback, qos_profile_sensor_data)

        self.temperature = W1ThermSensor()
        self.temp_file = temp_file

        self.temp_timer = self.create_timer(2.0, self.temp_read)
        self.last_position = None

    def temp_read(self):
        if self.last_position is not None:
            with open(self.temp_file, 'a') as fout:
                try:
                    fout.write(f"{self.temperature.get_temperature(Unit.DEGREES_F)} {self.last_position} {time.time()}\n")
                except:
                    print(f"update failed at {time.time()} {self.last_position}")

    def odom_callback(self, msg):
        self.last_position = msg.pose.pose.position


if __name__ == '__main__':
    msg_queue = Queue()
    print(f"Starting up {sys.argv[1]}...")
    runner.run_vision_node(lambda: TemperatureBot(msg_queue, f'/{sys.argv[1]}'), runner.OpenCvCode(0, lambda f, c, a: (f, None), None, msg_queue))
