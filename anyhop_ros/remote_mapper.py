import threading, subprocess, sys, math, curses, pickle, datetime

from pyhop_anytime import *

from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from runner import RemoteNode, straight_twist, turn_twist

def spin_thread(finished, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
    node.reset()
    rclpy.shutdown()


CLOSE_THRESHOLD = 0.5


def reset_pos(bot):
    call = f'ros2 service call /{bot}/reset_pose irobot_create_msgs/srv/ResetPose '
    call += '"{pose:{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"'
    return subprocess.run(call, shell=True, capture_output=True)


# From: https://stackoverflow.com/questions/21784625/how-to-input-a-word-in-ncurses-screen
def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo()
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row, col + 1 + len(prompt_string), 20)
    return text


class Runner:
    def __init__(self, stdscr):
        self.graph = Graph()
        self.last_position = None
        self.last_orientation = None
        self.last_name = None

        self.bot = sys.argv[1]
        self.stdscr = stdscr

        self.finished = threading.Event()
        self.cmd_queue = Queue(maxsize=1)
        self.pos_queue = Queue(maxsize=10)

        self.st = threading.Thread(target=spin_thread, args=(self.finished, lambda: RemoteNode(self.cmd_queue, self.pos_queue, f"/{self.bot}")))
        self.running = True

    def main_loop(self):
        self.st.start()
        self.stdscr.addstr(0, 0, 'WASD to move; R to reset position; X to record location; Q to quit')
        self.stdscr.refresh()

        self.stdscr.nodelay(True)
        self.input_window = curses.newwin(2, 80, 8, 0)

        while self.running: 
            try:
                self.handle_key()
            except curses.error:
                self.no_key()

        self.finished.set()
        self.st.join()
    
        with open(f"map_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'wb') as file:
            pickle.dump(self.graph, file)

    def handle_key(self):
        k = self.stdscr.getkey()
        if k == 'q':
            self.running = False
        elif k == 'x':
            self.input_window.clear()
            self.input_window.refresh()
            self.stdscr.addstr(7, 0, f"                                       ")
            name = my_raw_input(self.input_window, 0, 0, "Enter name:").lower().strip()
            name = name.decode('utf-8')
            self.stdscr.addstr(7, 0, f"Using {name}")
            self.graph.add_node(name, (self.last_position.x, self.last_position.y))
            if self.last_name is not None:
                self.graph.add_edge(name, self.last_name)
            self.last_name = name
        elif k == 'r':
            self.stdscr.addstr(1, 0, f"Waiting for reset...{' ' * 30}")
            result = reset_pos(self.bot)
            if result.returncode == 0:
                self.stdscr.addstr(1, 0, "Reset complete.     ")
            else:
                self.stdscr.addstr(1, 0, "Trouble with reset. ")
            self.stdscr.refresh()
        elif not self.cmd_queue.full():
            self.cmd_queue.put(k)

    def no_key(self):
        if not self.pos_queue.empty():
            pos = self.pos_queue.get()
            if type(pos) == float:
                self.stdscr.addstr(2, 0, f"{pos:7.2f} s")
            elif type(pos) == str:
                self.stdscr.addstr(6, 0, f"{pos}                          ")
            elif type(pos) == Pose:
                p = pos.position
                h = pos.orientation
                self.stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                self.stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
                closest = self.graph.closest_node_within(p.x, p.y, CLOSE_THRESHOLD)
                if closest is not None and closest != self.last_name:
                    if not self.graph.has_edge(self.last_name, closest):
                        self.graph.add_edge(self.last_name, closest)
                    self.last_name = closest
                self.stdscr.addstr(5, 0, f"Closest:     {closest}                                     ")
                self.last_position = p
                self.last_orientation = h
            self.stdscr.refresh()
    
        
def run_runner(stdscr):
    r = Runner(stdscr)
    curses.curs_set(0)
    stdscr.clear()
    r.main_loop()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_bot robot_name")
    else:
        curses.wrapper(run_runner)
