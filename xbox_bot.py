import runner
import sys
import time
import rclpy
import math
from evdev import InputDevice
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from queue import Queue
import threading
from rclpy.qos import qos_profile_sensor_data

# from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def yaw_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians


class XBoxReader:
    def __init__(self, msg_queue, incoming):
        self.msg_queue = msg_queue
        self.incoming = incoming

    def loop(self):
        dev = InputDevice('/dev/input/event0')
        for event in dev.read_loop():
            if event.value == 1:
                if event.code == 304:
                    self.msg_queue.put("A")
                elif event.code == 305:
                    self.msg_queue.put("B")
            if not self.incoming.empty():
                print("got message")
                break


class XBoxNode(runner.HdxNode):
    def __init__(self, msg_queue, namespace):
        super().__init__('x_box_for_real')
        self.msg_queue = msg_queue
        self.publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        self.create_timer(0.10, self.timer_callback)
        self.odometry = self.create_subscription(Odometry, f'/{namespace}/odom', self.odom_callback, qos_profile_sensor_data)
 

    def timer_callback(self):
        self.record_first_callback()
        if not self.msg_queue.empty():
            button = self.msg_queue.get()
            if button == 'A':
                self.publisher.publish(runner.straight_twist(0.5))
            elif button == 'B':
                self.publisher.publish(runner.turn_twist(math.pi / 4))

    def odom_callback(self, msg):
        ort = msg.pose.pose.orientation
        print(msg.pose.pose.position, yaw_from_quaternion(ort.x, ort.y, ort.z, ort.w))

    def add_self_recursive(self, executor):
        executor.add_node(self)


if __name__ == '__main__':
    rclpy.init()
    from_x = Queue()
    to_x = Queue()
    xboxer = XBoxReader(from_x, to_x)
    node = XBoxNode(from_x, 'tyrael')
    xbox_thread = threading.Thread(target=lambda x: x.loop(), args=(xboxer,))
    xbox_thread.start()
    runner.run_recursive_node(node)
    print("Runner done")
    to_x.put("QUIT")
