## TODO:
## * Create a plan to send the robot to any location using the
##   map_graph object as a data source.
##   * Create a plan sequencer to carry out the plan.
##   * The sequencer will display progress and allow 
##     stopping the robot prematurely as well.

import pickle, sys, curses
import threading, queue, subprocess
import traceback

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from runner import HdxNode, drain_queue


def main(stdscr):
    robot_name = sys.argv[2]
    with open(sys.argv[1], 'rb') as f:
        map_data = pickle.load(f)
        map_data.rotate = True
        map_graph = map_data.square_graph()

        curses.curs_set(0)

        cmd_queue = queue.Queue()
        pos_queue = queue.Queue()
        act_queue = queue.Queue()
        running = threading.Event()
        running.set()
        robot_thread = threading.Thread(target=spin_thread, args=(running, lambda: NavClient(cmd_queue, pos_queue, act_queue, f"/{robot_name}")))
        robot_thread.start()
        message = ""
        current_input = ""

        debug = ''

        current_location = None
        next_step = None
        goal = None
        pos = None
        action_msg = None

        stdscr.nodelay(True)

        while running.is_set():
            stdscr.clear()

            stdscr.addstr(0, 0, '"see [name]" to see coordinate; "go [name]" to go to a location; "reset" to reset position; "quit" to exit.')
            stdscr.addstr(1, 0, f"> {current_input}")
            stdscr.addstr(2, 0, message)
            stdscr.addstr(3, 0, f"{current_location}: {pos}")
            stdscr.addstr(4, 0, str(action_msg))
            stdscr.addstr(5, 0, debug)
        
            map_str = map_data.square_name_str()
            row = 6
            for i, line in enumerate(map_str.split('\n')):
                stdscr.addstr(row + i, 0, line)

            try:
                k = stdscr.getkey()
                debug = str(ord(k))
                curses.flushinp()
                if k == '\n':
                    if current_input == 'quit':
                        running.clear()
                    elif current_input == 'hi':
                        message = 'hello'
                    elif current_input.startswith('go'):
                        parts = current_input.split()
                        if len(parts) >= 2:
                            if parts[1] in map_graph:
                                goal = parts[1]
                                next_step = map_graph.next_step_from_to(current_location, goal)
                                cmd_queue.put(map_graph.node_value(next_step))
                            else:
                                message = f'unknown location: {parts[1]}'
                        else:
                            message = 'go where?'
                    elif current_input.startswith('see'):
                        parts = current_input.split()
                        if len(parts) >= 2:
                            if parts[1] in map_graph:
                                message = f"{map_graph.node_value(parts[1])}"
                            else:
                                message = "Unrecognized"
                        else:
                            message = 'see what?'
                    elif current_input == 'reset':
                        result = reset_pos(robot_name)
                        message = "complete" if result == 0 else "reset failed"
                    elif current_input == 'bye':
                        message = 'Type "quit" to exit'
                    else:
                        message = f"Unrecognized command: {current_input}"
                    
                    current_input = ''

                elif k == '\b':
                    current_input = current_input[:-1]
                else:
                    message = ''
                    current_input += k
            except curses.error as e:
                if str(e) != 'no input':
                    debug = traceback.format_exc()

            p = drain_queue(pos_queue)
            if p:
                pos = f"({p.position.x:.1f}, {p.position.y:.1f})"
                current_location, _ = map_graph.closest_node(p.position.x, p.position.y)

            a = drain_queue(act_queue)
            if a:
                action_msg = a
                if a == "goal reached":
                    if current_location == goal:
                        message = f"At goal {goal}!"
                    else:
                        next_step = map_graph.next_step_from_to(current_location, goal)
                        cmd_queue.put(map_graph.node_value(next_step))
        
        robot_thread.join()


def spin_thread(running, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and running.is_set() and not node.quitting():
        executor.spin_once()
    node.reset()
    rclpy.shutdown()


def reset_pos(bot):
    call = f'ros2 service call /{bot}/reset_pose irobot_create_msgs/srv/ResetPose '
    call += '"{pose:{position:{x: 0.0, y: 0.0, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"'
    return subprocess.run(call, shell=True, capture_output=True)


def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo()
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row, col + len(prompt_string) + 1, 20)
    text = text.decode('utf-8')
    return text


class NavClient(HdxNode):
    def __init__(self, cmd_queue, pos_queue, act_queue, namespace: str = ""):
        super().__init__('navigator')

        self.subscription = self.create_subscription(Odometry, f"{namespace}/odom", self.listener_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)
        self.cmd_queue = cmd_queue
        self.pos_queue = pos_queue
        self.act_queue = act_queue

        self.action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg.pose.pose)

    def timer_callback(self):
        msg = drain_queue(self.cmd_queue)
        if msg is not None:
            goal_msg = NavigateToPosition.Goal()
            goal_msg.achieve_goal_heading = True
            goal_msg.goal_pose = self.make_pose_from(msg)
            self.act_queue.put("action request sent")
            self.action_client.wait_for_server()
            self.act_queue.put("server response")
            future = self.action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.act_queue.put("goal accepted")
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.at_goal_callback)
        else:
            self.act_queue.put("goal rejected")

    def at_goal_callback(self, future):
        self.act_queue.put("goal reached")

    def make_pose_from(self, location):
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "test_id"
        p = Pose()
        p.position = Point()
        p.position.x, p.position.y = location
        p.position.z = 0.0
        p.orientation = Quaternion()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (0.0, 0.0, 0.0, 1.0)
        ps.pose = p
        return ps


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: curses_map.py pickled_map_file robot_name")
    else:
        curses.wrapper(main)
            
