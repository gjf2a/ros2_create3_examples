import sys
import math
import runner
import rclpy

from occupancy_grid import PathwayGrid

# Concept
#
# Builds a map by recording positions regularly.
# When it hits an obstacle, it aims for the most unknown area of the map.
# Once it reaches the unknown area, it selects a new unknown area and keeps moving.


class MapperNode(runner.HdxNode):
    def __init__(self, namespace: str):
        super().__init__('mapper_node', namespace)
        self.subscribe_hazard(self.bump_callback)
        self.subscribe_odom(self.odom_callback)
        self.map = PathwayGrid()
        self.bump = None
        self.turning = False

    def bump_clear(self):
        return self.bump is None

    def is_turning(self):
        return self.turning

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bump is None and not self.turning:
            self.bump = runner.find_bump_from(msg.detections)

