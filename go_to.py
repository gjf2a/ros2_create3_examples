import curses, threading, sys, math, queue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from runner import HdxNode, drain_queue, quaternion2euler, angle_diff, euclidean_distance, straight_twist, turn_twist

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


GO_TO_ANGLE_TOLERANCE = math.pi / 32
GO_TO_DISTANCE_TOLERANCE = 0.1

class GoToNode(HdxNode):
    def __init__(self, pos_queue: queue.Queue, cmd_queue: queue.Queue, status_queue: queue.Queue, namespace: str = ""):
        super().__init__('go_to', namespace)
        self.subscribe_odom(self.listener_callback)
        self.pos_queue = pos_queue
        self.cmd_queue = cmd_queue
        self.status_queue = status_queue
        self.active = False
        self.goal_orientation = None
        self.goal_position = None

    def listener_callback(self, pos: Odometry):
        self.pos_queue.put(pos)
        p = pos.pose.pose.position
        h = pos.pose.pose.orientation
        msg = drain_queue(self.cmd_queue)
        if msg is None:
            if self.active:
                euler = quaternion2euler(h)
                angle_disparity = angle_diff(self.goal_orientation, euler[0])
                distance = euclidean_distance(self.goal_position, (p.x, p.y))
                if abs(angle_disparity) > GO_TO_ANGLE_TOLERANCE:
                    sign = 1 if angle_disparity >= 0 else -1
                    self.send_twist(turn_twist(sign * math.pi / 4))
                    self.status_queue.put(f"Turning; sign is {sign}")
                elif distance > GO_TO_DISTANCE_TOLERANCE:
                    self.send_twist(straight_twist(0.5))
                    self.status_queue.put("Forward")
                else:
                    self.send_twist(straight_twist(0.0))
                    self.active = False
                    self.status_queue.put("Stopping")
            else:
                self.status_queue.put("Inactive")

        else:
            self.active = True
            self.goal_position = msg
            x, y = msg
            self.goal_orientation = math.atan2(y - p.y, x - p.x)
            self.status_queue.put(f"Received {msg}; goal orientation {self.goal_orientation}")


def printOdometry(stdscr, msg: Odometry):
    p = msg.pose.pose.position
    h = msg.pose.pose.orientation
    stdscr.addstr(2, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    stdscr.addstr(3, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})")


def main(stdscr):
    stdscr.nodelay(True)
    stdscr.clear()

    finished = threading.Event()
    ros_ready = threading.Event()
    pos_queue = queue.Queue()
    cmd_queue = queue.Queue()
    status_queue = queue.Queue()
    current_input = ''
    
    st = threading.Thread(target=spin_thread, args=(finished, ros_ready, lambda: GoToNode(pos_queue, cmd_queue, status_queue, "/archangel")))
    st.start()

    stdscr.addstr(0, 0, '"quit" to quit, "go x y" to drive somewhere')
    stdscr.refresh()
    
    while True:
        try:
            k = stdscr.getkey()
            curses.flushinp()
            if k == '\n':
                if current_input == 'quit':
                    break
                elif current_input.startswith("go"):
                    x, y = [float(n) for n in current_input.split()[1:]]
                    cmd_queue.put((x, y))
                current_input = ''
            elif k == '\b':
                current_input = current_input[:-1]
            else:
                current_input += k
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(5, 0, traceback.format_exc())

        if ros_ready.is_set():
            stdscr.addstr(4, 0, "ROS2 ready")

        p = drain_queue(pos_queue)
        if p:
            printOdometry(stdscr, p)

        s = drain_queue(status_queue)
        if s:
            stdscr.addstr(6, 0, f"{s}                                    ")
        stdscr.addstr(1, 0, f"{current_input}                         ")
        stdscr.refresh()
    finished.set()
    st.join()
    

if __name__ == '__main__':
    curses.wrapper(main)
