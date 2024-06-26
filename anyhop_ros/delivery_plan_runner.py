import pickle, curses, threading, queue, sys, copy
import rclpy
from nav_msgs.msg import Odometry
from runner import GoToNode, drain_queue
from pyhop_anytime import State
from pyhop_anytime_examples.graph_package_world import make_graph_planner
import RPi.GPIO as GPIO


def spin_thread(finished, ros_ready, node_maker):
    rclpy.init(args=None)
    executor = rclpy.get_global_executor()
    node = node_maker()
    executor.add_node(node)
    while executor.context.ok() and not finished.is_set() and not node.quitting():
        executor.spin_once()
        if node.ros_issuing_callbacks():
            ros_ready.set()
    node.reset()
    rclpy.shutdown()


def print_odometry(stdscr, msg: Odometry):
    p = msg.pose.pose.position
    h = msg.pose.pose.orientation
    stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")


def main(stdscr):
    with open(sys.argv[2], 'rb') as f:
        map_data = pickle.load(f)
        run_robot_map(stdscr, sys.argv[2], sys.argv[1], map_data)


class PlanManager:
    def __init__(self, holding):
        self.holding = holding
        self.planner = make_graph_planner()
        self.plan = None
        self.states = None
        self.current_step = None

    def make_delivery_plan(self, state: State):
        plan_times = self.planner.anyhop_random_tracked(state, [('deliver_all_packages_from', state.at)], 5)
        self.plan = plan_times[-1][0]
        self.states = self.planner.plan_states(state, self.plan)
        self.current_step = 0

    def current_action(self):
        return self.plan[self.current_step]

    def state_before_action(self):
        return self.states[self.current_step]

    def state_after_action(self):
        return self.states[self.current_step + 1]

    def next_location(self):
        return self.state_after_action().at

    def picked_up(self):
        return self.current_action()[0] == 'pick_up' and self.holding.is_set()

    def placed_down(self):
        return self.current_action()[0] == 'put_down' and not self.holding.is_set()

    def check_step(self, state):
        if self.current_action()[0] == 'move_one_step' and state.at == self.next_location():
            self.current_step += 1
        elif self.picked_up() or self.placed_down():
            state.package_locations = copy.deepcopy(self.state_after_action().package_locations)
            self.current_step += 1

    def plan_active(self):
        return self.current_step is not None and self.current_step < len(self.plan)


def holding_thread(finished, holding):
    # Inspired by: https://www.perplexity.ai/search/write-python-code-pDoDr1FKQVuJSK.sr8dezw

    GPIO.setmode(GPIO.BCM)
    BUMP_SENSOR_PIN = 17
    GPIO.setup(BUMP_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    while not finished.is_set():
        reading = GPIO.input(BUMP_SENSOR_PIN)
        if reading == 1:
            holding.set()
        else:
            holding.clear()

    GPIO.cleanup()


def run_robot_map(stdscr, filename, robot, map_data):
    state = State(filename)
    state.graph = map_data.square_graph()
    state.at = '$$'
    state.capacity = 1
    state.holding = []
    state.package_locations = {}
    state.package_goals = {}

    stdscr.nodelay(True)
    stdscr.clear()
    curses.curs_set(0)

    finished = threading.Event()
    ros_ready = threading.Event()
    active = threading.Event()
    holding = threading.Event()
    pos_queue = queue.Queue()
    cmd_queue = queue.Queue()
    status_queue = queue.Queue()
    manager = PlanManager(holding)
    current_input = ''
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: GoToNode(pos_queue, cmd_queue, status_queue, active, robot)))
    st.start()

    ht = threading.Thread(target=holding_thread, args=(finished, holding))
    ht.start()

    next_step = None
    goal = None

    stdscr.addstr(0, 0, '"quit" to quit, "stop" to stop, "go [name]" to go to a location; "reset" to reset odometry; "see [name]" to see coordinate')
    stdscr.addstr(1, 0, '"at [item] [location]..." to declare 1+ items at locations; "deliver [item] [location]..." to deliver 1+ items')
    map_str = map_data.square_name_str()
    for i, line in enumerate(map_str.split('\n')):
        stdscr.addstr(11 + i, 0, line)
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            curses.flushinp()
            if k == '\n':
                if current_input == 'quit':
                    break
                elif current_input.startswith('see'):
                    parts = current_input.split()
                    if len(parts) >= 2:
                        stdscr.addstr(6, 0, f"{state.graph.node_value(parts[1])}" if parts[1] in state.graph else 'Unrecognized')
                    else:
                        stdscr.addstr(6, 0, "see what?")
                elif current_input == 'stop':
                    drain_queue(cmd_queue)
                    active.clear()
                elif current_input == 'reset':
                    drain_queue(cmd_queue)
                    active.clear()
                    cmd_queue.put('reset')
                elif current_input.startswith("go"):
                    parts = current_input.split()
                    if len(parts) >= 2:
                        if parts[1] in state.graph:
                            goal = parts[1]
                            next_step = state.graph.next_step_from_to(state.at, goal)
                            cmd_queue.put(state.graph.node_value(next_step))
                            stdscr.addstr(6, 0, f'sent request "{current_input}"                ')
                        else:
                            stdscr.addstr(6, 0, f'Unknown location: {parts[1]}')
                    else:
                        stdscr.addstr(6, 0, 'go where?')
                elif current_input.startswith("at"):
                    parts = current_input.split()
                    if len(parts) >= 3 and len(parts) % 2 == 1:
                        for i in range(1, len(parts), 2):
                            item_name = parts[i]
                            item_location = parts[i + 1]
                            if item_location in state.graph:
                                state.package_locations[item_name] = item_location
                                stdscr.addstr(9, 0, str(state.package_locations))
                            else:
                                stdscr.addstr(6, 0, f'Unrecognized location: {item_location}')
                                break
                    else:
                        stdscr.addstr(6, 0, 'at: wrong # arguments')
                elif current_input.startswith("deliver"):
                    parts = current_input.split()
                    if len(parts) >= 3 and len(parts) % 2 == 1:
                        state.package_goals = {}
                        for i in range(1, len(parts), 2):
                            item_name = parts[i]
                            item_location = parts[i + 1]
                            if item_location in state.graph:
                                state.package_goals[item_name] = item_location
                            else:
                                stdscr.addstr(6, 0, f'Unrecognized location: {item_location}')
                                break
                        manager.make_delivery_plan(state)
                        if state.at != manager.next_location():
                            cmd_queue.put(state.graph.node_value(manager.next_location()))
                    else:
                        stdscr.addstr(6, 0, 'deliver: wrong # arguments')
                else:
                    stdscr.addstr(6, 0, f'Unrecognized input: "{current_input}"')
                current_input = ''
            elif k == '\b':
                current_input = current_input[:-1]
            else:
                current_input += k
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(6, 0, traceback.format_exc())

        if manager.plan_active():
            manager.check_step(state)
            stdscr.addstr(9, 0, str(state.package_locations))
            stdscr.addstr(10, 0, "Plan running   ")
        else:
            stdscr.addstr(10, 0, "No plan running")

        if ros_ready.is_set():
            stdscr.addstr(5, 0, "ROS2 ready")

        p = drain_queue(pos_queue)
        if p:
            print_odometry(stdscr, p)
            state.at, _ = state.graph.closest_node(p.pose.pose.position.x, p.pose.pose.position.y)

        s = drain_queue(status_queue)
        if s:
            stdscr.addstr(7, 0, f"{s}                                                ")
            if s == 'Stopping':
                if manager.plan_active():
                    if state.at != manager.next_location():
                        cmd_queue.put(state.graph.node_value(manager.next_location()))
                elif state.at != goal:
                    next_step = state.graph.next_step_from_to(state.at, goal)
                    cmd_queue.put(state.graph.node_value(next_step))
                    stdscr.addstr(5, 0, f'Sent next step: {next_step}')
        stdscr.addstr(2, 0, f"> {current_input}                                 ")
        stdscr.addstr(8, 0, f"{'active  ' if active.is_set() else 'inactive'}")
        stdscr.addstr(5, 0, f"@{state.at}; heading to {goal} via {next_step}        ")
        stdscr.refresh()
    finished.set()
    ht.join()
    st.join()
    

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 delivery_plan_runner.py robot pickled_map_file")
    else:
        curses.wrapper(main)
