## TODO:
## * Set up a ROS connection
##   * Display current location information from the robot
## * Use the NavClient class to send a robot to a location
##   * Only allow this if it is adjacent to the new location
## * Create a plan to send the robot to any location using the
##   map_graph object as a data source.
##   * Create a plan sequencer to carry out the plan.
##   * The sequencer will display progress and allow 
##     stopping the robot prematurely as well.

import pickle, sys, curses
import threading, queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from runner import straight_twist, turn_twist, HdxNode, drain_queue


def main(stdscr):
    robot_name = sys.argv[2]
    with open(sys.argv[1], 'rb') as f:
        map_data = pickle.load(f)
        map_data.rotate = True
        map_graph = map_data.square_graph()

        curses.curs_set(0)

        cmd_queue = queue.Queue()
        pos_queue = queue.Queue()
        running = threading.Event()
        running.set()
        robot_thread = threading.Thread(target=spin_thread, args=(running, lambda: NavClient(cmd_queue, pos_queue, f"/{robot_name}")))
        robot_thread.start()
        message = ""

        while running.is_set():
            stdscr.clear()

            stdscr.addstr(0, 0, 'Enter location name to see coordinate; "pos" for robot position; "go [name]" to go to a location; "reset" to reset position; "quit" to exit.')
            stdscr.addstr(2, 0, message)
        
            map_str = map_data.square_name_str()
            row = 4
            for i, line in enumerate(map_str.split('\n')):
                stdscr.addstr(row + i, 0, line)
        
            choice = my_raw_input(stdscr, 1, 0, "> ")

            if choice == 'quit':
                running.clear()
            elif choice == 'pos':
                pos = drain_queue(pos_queue)
                message = str(pos)
            elif choice == 'reset':
                result = reset_pos(robot_name)
                message = "complete" if result == 0 else "reset failed"
            elif map_graph.has_node(choice):
                message = f"{map_graph.node_value(choice)}"
            else:
                message = "Unrecognized"

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
    def __init__(self, cmd_queue, pos_queue, namespace: str = ""):
        super().__init__('navigator')

        self.subscription = self.create_subscription(Odometry, f"{namespace}/odom", self.listener_callback, qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_callback)
        self.cmd_queue = cmd_queue
        self.pos_queue = pos_queue

        self.action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')

    def listener_callback(self, msg: Odometry):
        self.pos_queue.put(msg.pose.pose)

    def timer_callback(self):
        #self.pos_queue.put(self.elapsed_time())
        msg = drain_queue(self.cmd_queue)
        if msg is not None:
            goal_msg = NavigateToPosition.Goal()
            goal_msg.achieve_goal_heading = True
            goal_msg.goal_pose = self.make_pose_from(msg)
            # TODO: Send a notification via pos_queue, but in a way that it doesn't immediately get overwritten.
            self.action_client.wait_for_server()
            # TODO: Send a follow-up message that the server responded
            future = self.action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            # TODO: Send a notification that the goal is accepted.
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.at_goal_callback)
        else:
            pass
            # TODO: Send a notification of a rejected goal

    def at_goal_callback(self, future):
        pass
        # TODO: Send a notification that the goal is reached.

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: curses_map.py pickled_map_file robot_name")
    else:
        curses.wrapper(main)
            
