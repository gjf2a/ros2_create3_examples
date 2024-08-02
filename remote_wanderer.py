import runner
import ir_bump_turn_odom, bump_turn_odom
from geometry_msgs.msg import Pose
import curses
import threading
from queue import Queue
import sys


class RemoteWandererNode(runner.RemoteNode):
    """
    Variant of RemoteNode that can be commanded to wander autonomously.
    """
    def __init__(self, cmd_queue, pos_queue, ir_queue, bump_queue, namespace: str = "", ir_limit=50):
        super().__init__(cmd_queue, pos_queue, ir_queue, bump_queue, namespace)
        self.wanderer = ir_bump_turn_odom.IrBumpTurnBot(namespace, ir_limit)
        self.add_child_node(self.wanderer)

    def timer_callback(self):
        self.pos_queue.put(self.elapsed_time())
        msg = runner.drain_queue(self.cmd_queue)
        if msg is not None:
            if msg in self.commands:
                self.wanderer.pause()
                self.publish_twist(self.commands[msg])
            elif msg == 'f':
                self.wanderer.resume()
        self.pos_queue.put(f"wanderer paused? {self.wanderer.paused} ")


def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    running = threading.Event()
    running.set()
    cmd_queue = Queue()
    pos_queue = Queue()
    bump_queue = Queue()
    ir_queue = Queue()
    bump_list = []

    st = threading.Thread(target=runner.spin_thread_recursive_node,
                          args=(running, lambda: RemoteWandererNode(cmd_queue, pos_queue, ir_queue, bump_queue,
                                                                    sys.argv[1])))
    st.start()

    stdscr.nodelay(True)
    stdscr.addstr(1, 0, 'WASD to move; F to Freely Wander; Q to quit')
    stdscr.refresh()

    while running.is_set():
        get_cmd(stdscr, cmd_queue, running)
        display_pose(stdscr, pos_queue, 2)
        display_ir(stdscr, ir_queue, 7)
        display_bump(stdscr, bump_queue, bump_list, 8)
        stdscr.refresh()

    st.join()


def get_cmd(stdscr, cmd_queue, running):
    try:
        k = stdscr.getkey()
        curses.flushinp()
        if k == 'q':
            running.clear()
        elif not cmd_queue.full():
            cmd_queue.put(k)
    except curses.error:
        pass


def display_pose(stdscr, pos_queue, start_line):
    pose = runner.drain_queue(pos_queue)
    if pose:
        if type(pose) == str:
            stdscr.addstr(start_line + 3, 0, f"str: {pose}")
        if type(pose) == Pose:
            p = pose.position
            h = pose.orientation
            stdscr.addstr(start_line + 1, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
            stdscr.addstr(start_line + 2, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
        elif type(pose) == float:
            stdscr.addstr(start_line, 0, f"{pose:.2f}")
        else:
            stdscr.addstr(start_line + 4, 0, f"{type(pose)} {pose}")
        return pose


def display_ir(stdscr, ir_queue, start_line):
    ir = runner.drain_queue(ir_queue)
    if ir:
        stdscr.addstr(start_line, 0, f"ir: {ir}{' ' * 30}")
        return ir


def display_bump(stdscr, bump_queue, bump_list, start_line):
    if not bump_queue.empty():
        b = bump_queue.get()
        bump_list.append(b)
        stdscr.addstr(start_line, 0, f"{bump_list[-3:]}")
        return b


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_wanderer robot_name")
    else:
        curses.wrapper(main)
