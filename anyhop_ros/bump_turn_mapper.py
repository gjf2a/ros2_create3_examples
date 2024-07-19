import sys, queue, threading, curses, pickle, datetime
import runner
from nav_msgs.msg import Odometry

from occupancy_grid import PathwayGrid


class MapperNode(runner.HdxNode):
    def __init__(self, map_str_queue: queue.Queue, namespace: str):
        super().__init__('mapper_node', namespace)
        self.map_str_queue = map_str_queue
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

        self.map_str_queue.put((self.last_pose, self.map))

    def bump_callback(self, msg):
        self.record_first_callback()
        if self.bump is None and not self.turning:
            self.bump = runner.find_bump_from(msg.detections)
            if self.bump:
                x, y = self.last_x_y()
                self.goal = self.map.centroid_of_open_space(x, y, 4)


class Runner:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.bot = sys.argv[1]
        self.last_map = None
        self.last_pos = None
        self.map_queue = queue.Queue()
        self.running = threading.Event()
        self.robot_thread = threading.Thread(target=runner.spin_thread_simple,
                                             args=(self.running, lambda: MapperNode(self.map_queue, f"/{self.bot}")))

    def main_loop(self):
        self.running.set()
        self.stdscr.addstr(0, 0, 'Q to quit')
        self.stdscr.refresh()
        self.stdscr.nodelay(True)
        self.robot_thread.start()

        while self.running.is_set():
            try:
                k = self.stdscr.getkey()
                curses.flushinp()
                if k == 'q':
                    self.running.clear()
            except curses.error:
                pass
            self.handle_info()

        self.robot_thread.join()
        with open(f"map_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'wb') as file:
            pickle.dump(self.last_map, file)

    def handle_info(self):
        info = runner.drain_queue(self.map_queue)
        if info:
            self.last_pos, self.last_map = info
            p = self.last_pos.position
            h = self.last_pos.orientation
            self.stdscr.addstr(1, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
            self.stdscr.addstr(2, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
            map_str = self.last_map.square_name_str()
            for i, line in enumerate(map_str.split()):
                self.stdscr.addstr(3 + i, 0, line)
            self.stdscr.refresh()


def run_runner(stdscr):
    r = Runner(stdscr)
    curses.curs_set(0)
    stdscr.clear()
    r.main_loop()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: bump_turn_mapper robot_name")
    else:
        curses.wrapper(run_runner)

