import runner
import ir_bump_turn_odom
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

    def timer_callback(self):
        self.pos_queue.put(self.elapsed_time())
        msg = runner.drain_queue(self.cmd_queue)
        if msg is not None:
            if msg in self.commands:
                self.wanderer.pause()
                self.publish_twist(self.commands[msg])
            elif msg == 'f':
                self.wanderer.resume()

    def add_self_recursive(self, executor):
        executor.add_node(self)
        self.wanderer.add_self_recursive(executor)


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
        try:
            k = stdscr.getkey()
            if k == 'q':
                break
            elif not cmd_queue.full():
                cmd_queue.put(k)
        except curses.error:
            pass

        pose = runner.drain_queue(pos_queue)
        if pose:
            if type(pose) == str:
                stdscr.addstr(5, 0, f"str: {pose}")
            if type(pose) == Pose:
                p = pose.position
                h = pose.orientation
                stdscr.addstr(3, 0, f"Position:    ({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})        ")
                stdscr.addstr(4, 0, f"Orientation: ({h.x:6.2f}, {h.y:6.2f}, {h.z:6.2f}, {h.w:6.2f})        ")
            elif type(pose) == float:
                stdscr.addstr(2, 0, f"{pose:.2f}")
            else:
                stdscr.addstr(7, 0, f"{type(pose)} {pose}")

        ir = runner.drain_queue(ir_queue)
        if ir:
            stdscr.addstr(6, 0, f"ir: {ir}{' ' * 30}")

        if not bump_queue.empty():
            b = bump_queue.get()
            bump_list.append(b)
            stdscr.addstr(8, 0, f"{bump_list}")
        stdscr.refresh()

    running.clear()
    st.join()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: remote_wanderer robot_name")
    else:
        curses.wrapper(main)
