import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import ResetPose

from geometry_msgs.msg import Pose

class MinimalService(Node):
    def __init__(self):
        super().__init__('reset_pos_test')
        self.srv = self.create_service(ResetPose, '/archangel/reset_pose', self.reset_pose_callback)

    def reset_pose_callback(self, request, response):
