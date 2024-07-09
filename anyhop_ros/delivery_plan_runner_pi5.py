import pickle, curses, threading, queue, sys, copy
import rclpy
from nav_msgs.msg import Odometry
from runner import GoToNode, drain_queue
from pyhop_anytime import State
from pyhop_anytime_examples.graph_package_world import make_graph_planner
from gpiozero import Button


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
    curses.curs_set(0)
    with open(sys.argv[2], 'rb') as f:
        map_data = pickle.load(f)
        runner = RobotMapRunner(stdscr, sys.argv[2], sys.argv[1], map_data)
        runner.run_loop()


class PlanManager:
    def __init__(self, holding):
        self.holding = holding
        self.planner = make_graph_planner()
        self.plan = None
        self.states = None
        self.current_step = None

    def make_delivery_plan(self, state: State):
        plan_times = self.planner.anyhop_random_tracked(state, [('deliver_all_packages_from', state.at)], 3)
        self.plan = plan_times[-1][0]
        self.states = self.planner.plan_states(state, self.plan)
        self.current_step = 0

    def make_travel_plan(self, state: State, goal: str):
        plan_times = self.planner.anyhop_random_tracked(state, [('go_to', goal)], 3)
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
        updated = False
        if self.current_action()[0] == 'move_one_step' and state.at == self.next_location():
            self.current_step += 1
            updated = True
        elif self.picked_up() or self.placed_down():
            state.package_locations = copy.deepcopy(self.state_after_action().package_locations)
            self.current_step += 1
            updated = True

        if updated and self.current_step >= len(self.plan):
            self.stop_plan()

        return updated

    def plan_active(self):
        return self.current_step is not None and self.current_step < len(self.plan)

    def stop_plan(self):
        self.current_step = None



def holding_thread(finished, holding):
    # Inspired by: https://www.perplexity.ai/search/write-python-code-pDoDr1FKQVuJSK.sr8dezw
    # Adapted for pi5 with https://gpiozero.readthedocs.io/en/latest/recipes.html

    button = Button(17)
    while not finished.is_set():
        if button.is_pressed:
            holding.set()
        else:
            holding.clear()

    GPIO.cleanup()


class RobotMapRunner:
    def __init__(self, stdscr, filename, robot, map_data):
        self.state = State(filename)
        self.state.graph = map_data.square_graph()
        self.state.at = '$$'
        self.state.capacity = 1
        self.state.holding = []
        self.state.package_locations = {}
        self.state.package_goals = {}

        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0,
                      '"quit" to quit, "stop" to stop, "go [name]" to go to a location; "reset" to reset odometry; "see [name]" to see coordinate')
        stdscr.addstr(1, 0,
                      '"at [item] [location]..." to declare 1+ items at locations; "deliver [item] [location]..." to deliver 1+ items')
        map_str = map_data.square_name_str()
        for i, line in enumerate(map_str.split('\n')):
            stdscr.addstr(11 + i, 0, line)
        stdscr.refresh()
        self.stdscr = stdscr

        self.finished = threading.Event()
        self.ros_ready = threading.Event()
        self.active = threading.Event()
        self.holding = threading.Event()
        self.pos_queue = queue.Queue()
        self.cmd_queue = queue.Queue()
        self.status_queue = queue.Queue()
        self.manager = PlanManager(self.holding)
        self.current_input = ''

        self.holding.clear()

        self.st = threading.Thread(target=spin_thread, args=(self.finished, self.ros_ready,
                                                             lambda: GoToNode(self.pos_queue, self.cmd_queue,
                                                                              self.status_queue, self.active, robot)))
        self.ht = threading.Thread(target=holding_thread, args=(self.finished, self.holding))

    def running_plan(self):
        return self.manager.plan_active()

    def run_loop(self):
        self.st.start()
        self.ht.start()

        while not self.finished.is_set():
            try:
                k = self.stdscr.getkey()
                curses.flushinp()
                if k == '\n':
                    self.dispatch_command()
                    self.current_input = ''
                elif k == '\b':
                    self.current_input = self.current_input[:-1]
                else:
                    self.current_input += k
            except curses.error as e:
                if str(e) != 'no input':
                    self.stdscr.addstr(6, 0, traceback.format_exc())

            if self.ros_ready.is_set():
                self.stdscr.addstr(5, 0, "ROS2 ready")

            self.show_plan_status()
            self.odometry_update()
            self.status_update()
            self.other_update()
            self.stdscr.refresh()

        self.ht.join()
        self.st.join()

    def dispatch_command(self):
        if self.current_input == 'quit':
            self.finished.set()
        elif self.current_input.startswith('see'):
            self.see()
        elif self.current_input == 'stop':
            self.stop()
        elif self.current_input == 'reset':
            self.stop()
            self.cmd_queue.put('reset')
        elif self.current_input.startswith("go"):
            self.go()
        elif self.current_input.startswith("at"):
            self.at()
        elif self.current_input.startswith("deliver"):
            self.deliver()
        else:
            self.stdscr.addstr(6, 0, f'Unrecognized input: "{self.current_input}"')

    def stop(self):
        self.manager.stop_plan()
        drain_queue(self.cmd_queue)
        self.active.clear()

    def see(self):
        parts = self.current_input.split()
        if len(parts) >= 2:
            location = f"{self.state.graph.node_value(parts[1])}" if parts[1] in self.state.graph else 'Unrecognized'
            self.stdscr.addstr(6, 0, location)
        else:
            self.stdscr.addstr(6, 0, "see what?")

    def go(self):
        parts = self.current_input.split()
        if len(parts) >= 2:
            if parts[1] in self.state.graph:
                if self.running_plan():
                    self.stop()
                self.manager.make_travel_plan(self.state, parts[1])
                self.next_plan_step('go_to')
            else:
                self.stdscr.addstr(6, 0, f'Unknown location: {parts[1]}')
        else:
            self.stdscr.addstr(6, 0, 'go where?')

    def at(self):
        parts = self.current_input.split()
        if len(parts) >= 3 and len(parts) % 2 == 1:
            for i in range(1, len(parts), 2):
                item_name = parts[i]
                item_location = parts[i + 1]
                if item_location in self.state.graph:
                    self.state.package_locations[item_name] = item_location
                    self.stdscr.addstr(9, 0, str(self.state.package_locations))
                else:
                    self.stdscr.addstr(6, 0, f'Unrecognized location: {item_location}')
                    break
        else:
            self.stdscr.addstr(6, 0, 'at: wrong # arguments')

    def deliver(self):
        parts = self.current_input.split()
        if len(parts) >= 3 and len(parts) % 2 == 1:
            self.state.package_goals = {}
            for i in range(1, len(parts), 2):
                item_name = parts[i]
                item_location = parts[i + 1]
                if item_location in self.state.graph:
                    self.state.package_goals[item_name] = item_location
                else:
                    self.stdscr.addstr(6, 0, f'Unrecognized location: {item_location}')
                    break
            if self.running_plan():
                self.stop()
            self.manager.make_delivery_plan(self.state)
            self.next_plan_step('deliver')
        else:
            self.stdscr.addstr(6, 0, 'deliver: wrong # arguments')

    def show_plan_status(self):
        self.stdscr.addstr(9, 0, str(self.state.package_locations))
        if self.running_plan():
            self.stdscr.addstr(10, 0,
                               f"Plan running; step {self.manager.current_step}  {self.manager.current_action()} ")
            if self.manager.check_step(self.state):
                self.next_plan_step('check')
        else:
            self.stdscr.addstr(10, 0, f"No plan running{' ' * 40}")

    def odometry_update(self):
        p = drain_queue(self.pos_queue)
        if p:
            print_odometry(self.stdscr, p)
            self.state.at, _ = self.state.graph.closest_node(p.pose.pose.position.x, p.pose.pose.position.y)

    def status_update(self):
        s = drain_queue(self.status_queue)
        if s:
            self.stdscr.addstr(7, 0, f"{s}                                                ")

    def next_plan_step(self, tag):
        if self.running_plan() and self.state.at != self.manager.next_location():
            self.cmd_queue.put(self.state.graph.node_value(self.manager.next_location()))
            self.stdscr.addstr(6, 0, f'Sent next plan step: @{self.state.at} -> {self.manager.next_location()} ({tag})')

    def other_update(self):
        self.stdscr.addstr(2, 0, f"> {self.current_input}                                 ")
        self.stdscr.addstr(8, 0, f"{'active  ' if self.active.is_set() else 'inactive'}")
        if self.manager.plan_active():
            self.stdscr.addstr(5, 0, f"@{self.state.at}; heading towards {self.manager.next_location()}")
        else:
            self.stdscr.addstr(5, 0, f"@{self.state.at}{' ' * 40}")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 delivery_plan_runner.py robot pickled_map_file")
    else:
        curses.wrapper(main)
