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


def main():
    with open(sys.argv[2], 'rb') as f:
        map_data = pickle.load(f)
        runner = RobotMapRunner(sys.argv[2], sys.argv[1], map_data)
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


class RobotMapRunner:
    def __init__(self, filename, robot, map_data, map_description):
        self.state = State(filename)
        self.state.description = map_description
        self.state.graph = map_data.square_graph()
        self.state.at = '$$'
        self.state.capacity = 1
        self.state.holding = []
        self.state.package_locations = {}
        self.state.package_goals = {}

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
            print("Unrecognized input: " + self.current_input)

    def stop(self):
        self.manager.stop_plan()
        drain_queue(self.cmd_queue)
        self.active.clear()

    def see(self):
        parts = self.current_input.split()
        if len(parts) >= 2:
            location = f"{self.state.graph.node_value(parts[1])}" if parts[1] in self.state.graph else 'Unrecognized'
            print(location)
        else:
            print("Object not found")

    def go(self):
        parts = self.current_input.split()
        if len(parts) >= 2:
            if parts[1] in self.state.graph:
                if self.running_plan():
                    self.stop()
                self.manager.make_travel_plan(self.state, parts[1])
                self.next_plan_step('go_to')
            else:
                print("unkown location")
        else:
            print("no location")

    def at(self):
        parts = self.current_input.split()
        if len(parts) >= 3 and len(parts) % 2 == 1:
            for i in range(1, len(parts), 2):
                item_name = parts[i]
                item_location = parts[i + 1]
                if item_location in self.state.graph:
                    self.state.package_locations[item_name] = item_location
                else:
                    print("unknown location")
                    break
        else:
            print("wrong number arguments")

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
                    print("unrecognized location")
                    break
            if self.running_plan():
                self.stop()
            self.manager.make_delivery_plan(self.state)
            self.next_plan_step('deliver')
        else:
            print("wrong number of arguments")



if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 delivery_plan_runner.py robot pickled_map_file")
    else:
        main()
