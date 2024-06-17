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
from runner import GoToNode, drain_queue


def main(stdscr):
    robot_name = f"/{sys.argv[2]}"
    with open(sys.argv[1], 'rb') as f:
        map_data = pickle.load(f)
        map_data.rotate = True
        map_graph = map_data.square_graph()

        curses.curs_set(0)

        cmd_queue = queue.Queue()
        pos_queue = queue.Queue()
        status_queue = queue.Queue()
        go_to_active = threading.Event()
        running = threading.Event()
        running.set()
        robot_thread = threading.Thread(target=spin_thread, args=(running, lambda: GoToNode(cmd_queue, pos_queue, status_queue, go_to_active, robot_name)))
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
            stdscr.addstr(2, 0, f"> {current_input}")
            stdscr.addstr(3, 0, message)
            stdscr.addstr(4, 0, f"{robot_name}@{current_location}: {pos}")
            stdscr.addstr(5, 0, str(action_msg))
            stdscr.addstr(6, 0, debug)
        
            map_str = map_data.square_name_str()
            row = 7
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
                                message = f'sent request "{current_input}"'
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
                        message = "complete" if result.returncode == 0 else f"reset failed: {result}"
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

            action_msg = drain_queue(status_queue)

            if goal is not None and not go_to_active.is_set():
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


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: curses_map.py pickled_map_file robot_name")
    else:
        curses.wrapper(main)
            
