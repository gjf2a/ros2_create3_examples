import math
import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
import runner
from rclpy.node import Node
import math, sys, threading

import Planner

class NavToActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Nav_To")
        self._action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')
        self.callback = callback
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.running = False
        self.avoid_angle = math.pi/2
        self.bump = None
        self.turning = False

    def bump_callback(self, msg):
        self.started = True
        if self.bump is None and not self.turning:
            self.bump = runner.find_bump_from(msg.detections)
            if self.bump:
                print(f"Detected {self.bump}")
   
    def send_goal(self, achieve_goal_heading = False, goal_pose = [[7.715,0.0,0.0], [0.0,0.0,0.0,1.0]]):
        self.running = True
        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = achieve_goal_heading
        ps = PoseStamped()
        ps.header = Header()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "test_id"
        p = Pose()
        p.position = Point()
        p.position.x, p.position.y, p.position.z = goal_pose[0]
        p.orientation = Quaternion()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = goal_pose[1]
        ps.pose = p
        goal_msg.goal_pose = ps
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("Goal rejected...")
        else:
            print("Goal accepted.")    
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.callback)

    def spin_thread(self):
        st = threading.Thread(target=lambda ac: rclpy.spin(ac), args=(self,))
        st.start()

def example_callback(future):
    print("Entering example_callback")
    result = future.result().result
    print("finished...", result)

def main(args=None, namespace=''):
    chosen_plan_steps = 999
    state = Planner.State('McRey')
    state.robots = ['robot1']
    state.visited = {'robot1': set()}
    state.loc = {'robot1': 'A'}
    state.connected = {
            'A': ['B'],
            'B': ['A', 'C', 'E'],
            'C': ['B', 'D', 'E'],
            'D': ['C'],
            'E': ['C', 'F', 'B'],
            'F': ['E']
        }
    
    planner = Planner.Planner()
    planner.declare_operators(Planner.move)
    planner.declare_methods(Planner.navigate)
    all_plans = (planner.anyhop(state, [('navigate','robot1','A','F'), ('navigate', 'robot1', 'F', 'D'), ('navigate','robot1','D','A')]))
    for plan in all_plans:
        if int(plan[1]) < chosen_plan_steps:
            chosen_plan_steps = plan[1]
            chosen_plan = plan


    global finish_flag
    rclpy.init(args=args)
    action_client = NavToActionClient(example_callback, namespace)
    action_client.spin_thread()
    for task in chosen_plan[0]:
        goal = task[3]
        if goal == 'A':
            #316
            action_client.send_goal(False, [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
            input("Hit Enter Upon Task Completion")
        elif goal == 'B':
            #Dr. Ferrer's Office Door
            action_client.send_goal(False, [[7.715,0.0,0.0], [0.0,0.0,0.0,1.0]])
            input("Hit Enter Upon Task Completion")
        elif goal == 'C':
            #Center of Long Hallway
            action_client.send_goal(False, [[7.715,10.6,0.0], [0.0,0.0,0.0,1.0]])
            # Unmeasured as of this time, to be checked
            input("Hit Enter Upon Task Completion")
        elif goal == 'D':
            #Lounge
            action_client.send_goal(False, [[3.215,10.6,0.0], [0.0,0.0,0.0,1.0]])
            #Unmeasured as of this time, to be checked
            input("Hit Enter Upon Task Completion")
        elif goal == 'E':
            #Dr. Goadrich's Office Door
            action_client.send_goal(False, [[7.715,21.445,0.0], [0.0,0.0,0.0,1.0]])
            input("Hit Enter Upon Task Completion")
        elif goal == 'F':
            #309 (I think that's the right room number)
            action_client.send_goal(False, [[1.515, 21.445,0.0], [0.0,0.0,0.0,1.0]])
            input("Hit Enter Upon Task Completion")

    rclpy.shutdown()
    print("done")

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()

