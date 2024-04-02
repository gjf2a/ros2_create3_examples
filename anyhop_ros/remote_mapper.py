import threading, subprocess, sys, math, curses, pickle, datetime

from pyhop_anytime import *

from queue import Queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from runner import HdxNode, straight_twist, turn_twist

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

class RemoteNode(HdxNode):
    def __init__(self, cmd_queue, pos_queue, namespace: str = ""):
        super().__init__('odometry_subscriber')

        self.commands = {
            'w': straight_twist(0.5),
            'a': turn_twist(math.pi/4),
            's': straight_twist(0.0),
            'd': turn_twist(-math.pi/4)
        }

        self.subscription = self.create_subscription(
            Odometry, namespace + '/odom', self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        self.create_timer(0.1, self.timer_callback)
        self.cmd_queue = cmd_queue
        self.pos_queue = pos_queue
        self.last_key = None

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg.pose.pose)

    def timer_callback(self):
        self.pos_queue.put(self.elapsed_time())
        if not self.cmd_queue.empty():
            msg = self.cmd_queue.get()
            self.last_key = msg
            if self.last_key in self.commands:
                self.publisher.publish(self.commands[msg])
                self.pos_queue.put(self.last_key)


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


def main(stdscr):
    graph = Graph()
    last_position = None
    last_orientation = None
    last_name = None


    bot = sys.argv[1]
    curses.curs_set(0)
    stdscr.clear()

    finished = threading.Event()
    cmd_queue = Queue(maxsize=1)
    pos_queue = Queue(maxsize=10)
    
    st = threading.Thread(target=spin_thread, args=(finished, lambda: RemoteNode(cmd_queue, pos_queue, f"/{bot}")))
    st.start()

    stdscr.addstr(0, 0, 'WASD to move; R to reset position; X to record location; Q to quit')
    stdscr.refresh()

    stdscr.nodelay(True)
    input_window = curses.newwin(2, 80, 8, 0)
    
    while True:
        try:
            k = stdscr.getkey()
            if k == 'q':
                break
            elif k == 'x':
                input_window.clear()
                input_window.refresh()
                stdscr.addstr(7, 0, f"                                       ")
                name = my_raw_input(input_window, 0, 0, "Enter name:").lower().strip()
                name = name.decode('utf-8')
                stdscr.addstr(7, 0, f"Using {name}")
                graph.add_node(name, (last_position.x, last_position.y))
                if last_name is not None:
                    graph.add_edge(name, last_name)
                last_name = name
            elif k == 'r':
                stdscr.addstr(1, 0, f"Waiting for reset...{' ' * 30}")
                result = reset_pos(bot)
                if result.returncode == 0:
                    stdscr.addstr(1, 0, "Reset complete.     ")
                else:
                    stdscr.addstr(1, 0, "Trouble with reset. ")
                stdscr.refresh()
            elif not cmd_queue.full():
                cmd_queue.put(k)
        except curses.error:
            # No key pressed
            if not pos_queue.empty():
                pos = pos_queue.get()
                if type(pos) == float:
                    stdscr.addstr(2, 0, f"{pos:7.2f} s")
                elif type(pos) == str:
                    stdscr.addstr(6, 0, f"{pos}                          ")
                elif type(pos) == Pose:
                    p = pos.position
                    h = pos.orientation
                    stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                    stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
                    closest = graph.closest_node_within(p.x, p.y, CLOSE_THRESHOLD)
                    if closest is not None and closest != last_name:
                        if not graph.has_edge(last_name, closest):
                            graph.add_edge(last_name, closest)
                        last_name = closest
                    stdscr.addstr(5, 0, f"Closest:     {closest}                                     ")
                    last_position = p
                    last_orientation = h
                stdscr.refresh()

            
    finished.set()
    st.join()
    with open(f"map_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'wb') as file:
        pickle.dump(graph, file)

    

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_bot robot_name")
    else:
        curses.wrapper(main)
