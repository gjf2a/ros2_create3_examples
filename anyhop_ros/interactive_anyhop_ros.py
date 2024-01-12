from curses_menu_demo import MenuItems

from pyhop_anytime import *

from curses import wrapper, curs_set, A_REVERSE, error
from queue import Queue
import copy, sys, threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import NavigateToPosition
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

def plan_manager_thread(map_info, incoming_queue, stdscr):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    manager = PlanManager(map_info, stdscr)
    executor.add_node(manager.action_client)
    while executor.context.ok():
        if not incoming_queue.empty():
            msg = incoming_queue.get()
            stdscr.add_str(0, 0, f"Received '{msg}' message")
            stdscr.refresh()
            if msg == 'quit':
                break
            elif msg in map_info and manager.available():
                manager.make_plan(msg)
                manager.execute_next_step()
        executor.spin_once()

    rclpy.shutdown()


class MapInfo:
    def __init__(self, filename=None):
        self.adjacency = {}
        self.locations = {}
        self.name = "Unnamed"
        if filename:
            with open(filename) as file:
                self.name = filename
                for line in file:
                    self.add_row(line)

    def __contains__(self, location):
        return location in self.locations

    def all_locations(self):
        return list(self.locations)

    def start(self):
        for location in self.locations:
            return location

    def add_row(self, row_str):
        name, location, adjacency = row_str.split(':')
        name = name.strip()
        x, y = location.strip()[1:-1].split(',')
        self.locations[name] = (float(x), float(y))
        for adj in adjacency.split(','):
            self.add_adjacent(name, adj)

    def add_adjacent(self, a, b):
        a = a.strip()
        b = b.strip()
        if a not in self.adjacency:
            self.adjacency[a] = []
        if b not in self.adjacency[a]:
            self.adjacency[a].append(b)
        if b not in self.adjacency or a not in self.adjacency[b]:
            self.add_adjacent(b, a)

    def __str__(self):
        s = ""
        for name in self.locations:
            s += f"{name}: {self.locations[name]}: "
            for adj in self.adjacency[name]:
                s += f'{adj}, '
            s = f"{s[:-2]}\n"
        return s

    def pyhop_state(self):
        state = pyhop.State(self.name)
        state.connected = copy.deepcopy(self.adjacency)
        state.visited = {'robot': set()}
        state.location = {'robot': self.start()}    
        return state


def make_planner():
    planner = pyhop.Planner()
    planner.declare_operators(go)
    planner.declare_methods(find_route)
    return planner


def go_pre(state, robot, start, end):
    return state.location[robot] == start and end in state.connected[start]


def go(state, robot, start, end):
    if go_pre(state, robot, start, end):
        state.location[robot] = end
        state.visited[robot].add(end)
        return state


def find_route(state, robot, start, end):
    if start == end:
        state.visited = {robot: set()}
        return TaskList(completed=True)
    elif go_pre(state, robot, start, end) and end not in state.visited[robot]:
        return TaskList([('go', robot, start, end)])
    else:
        tasks = []
        for neighbor in state.connected[start]:
            tasks.append([('go', robot, start, neighbor), 
                          ('find_route', robot, neighbor, end)])
        return TaskList(tasks)


class PlanManager:
    def __init__(self, map_info, stdscr, namespace='archangel', state=None):
        self.map_info = map_info
        planner = make_planner()
        if state is None:
            state = map_info.pyhop_state()
        self.state = state 
        self.plan = None
        self.action_client = NavClient("Navigator", self.step_done, namespace)
        self.plan_step = 0
        self.stdscr = stdscr

    def make_plan(self, goal, max_seconds=None):
        self.plan_step = 0
        self.plan = planner.anyhop_best(state, [('find_route', 'robot', state.location['robot'], goal)], max_seconds=max_seconds)

    def current_goal_name(self):
        return self.plan[self.plan_step][3]

    def current_goal_coordinates(self):
        return self.map_info.locations[self.current_goal_name()]

    def execute_next_step(self):
        self.stdscr.addstr(0, 0, f"Starting {self.plan[self.plan_step]}")
        self.stdscr.refresh()
        self.action_client.send_goal(self.current_goal_coordinates())
        
    def step_done(self, future):
        self.stdscr.addstr(0, 0, f"Completed {self.plan[self.plan_step]}")
        self.stdscr.refresh()
        # Ask anyhop to update self.state using the current operator
        self.plan_step += 1
        if not self.plan_complete():
            self.execute_next_step()

    def plan_complete(self):
        return self.plan_step == len(self.plan)

    def available(self):
        return self.plan is None or self.plan_complete()
        

class NavClient(Node):
    def __init__(self, name, callback, namespace):
        super().__init__(name)
        self.action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')
        self.running = False
        self.callback = callback

    def send_goal(self, goal):
        self.running = True
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = True
        goal_msg.goal_pose = self.make_pose_from(goal)
        
        print(f"Preparing to send goal message {goal_msg}")
        self.action_client.wait_for_server()
        print("Server responded - sending goal")
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            print("Goal accepted.")
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.callback)
        else:
            print("Goal rejected...")

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


def main(stdscr):
    namespace, filename = sys.argv[1:]
    map_info = MapInfo(filename=filename)
    locations = map_info.all_locations()
    to_manager = Queue(maxsize=1)

    st = threading.Thread(target=plan_manager_thread, args=(map_info, to_manager, stdscr))
    st.start()
    
    curs_set(0)
    stdscr.nodelay(True)
    menu = MenuItems(4, locations + ["quit"])
    stdscr.clear()
    update = True
    
    while True:
        if update:
            menu.show(stdscr)
            stdscr.refresh()
            update = False

        try:
            key = stdscr.getkey()
            update = True
            selection = menu.update_from_key(key)            
            if selection is not None:
                stdscr.addstr(1, 0, f"sending '{selection}'")
                stdscr.refresh()
                if not to_manager.full():
                    to_manager.put(selection)
                    if selection == "quit":
                        stdscr.addstr(2, 0, "quitting...")
                        stdscr.refresh()
                        break
        except error:
            pass

    st.join()
    


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: anyhop_ros namespace map_filename")
    else:
        wrapper(main)
