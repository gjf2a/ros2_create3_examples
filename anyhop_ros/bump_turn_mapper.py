import sys
import math
import runner
import rclpy
from nav_msgs.msg import Odometry

from occupancy_grid import PathwayGrid


class MapperNode(runner.HdxNode):
    def __init__(self, namespace: str):
        super().__init__('mapper_node', namespace)
        self.subscribe_hazard(self.bump_callback)
        self.subscribe_odom(self.odom_callback)
        self.map = PathwayGrid()
        self.bump = None
        self.turning = False
        self.goal = (-1, 0)
        self.last_pose = None

    def bump_clear(self):
        return self.bump is None

    def is_turning(self):
        return self.turning

    def last_x_y(self):
        p = self.last_pose.position
        return p.x, p.y

    def odom_callback(self, pos: Odometry):
        self.last_pose = pos.pose.pose
        x, y = self.last_x_y()
        self.map.visit(x, y)
        twist = None
        if self.goal is not None:
            twist = runner.twist_towards_goal(self.goal[0], self.goal[1], self.last_pose.position,
                                              self.last_pose.orientation)
        if twist:
            self.publish_twist(twist)
        else:
            self.goal = self.map.centroid_of_unvisited()

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bump is None and not self.turning:
            self.bump = runner.find_bump_from(msg.detections)
            if self.bump:
                x, y = self.last_x_y()
                self.goal = self.map.centroid_of_open_space(x, y, 4)

