import math
import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import NavigateToPosition
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.action import LedAnimation
from irobot_create_msgs.msg import LightringLeds, LedColor
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import qos_profile_sensor_data
import runner
from rclpy.node import Node
import math, sys, threading

import Planner

class RotateActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Demo")
        self._action_client = ActionClient(self, RotateAngle, f'{namespace}/rotate_angle')
        self.callback = callback
        
    def send_spin_goal(self, goal_heading, radians_per_sec=1.0):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = goal_heading
        goal_msg.max_rotation_speed = radians_per_sec
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



class NavToActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Nav_To")
        self._action_client = ActionClient(self, NavigateToPosition, f'{namespace}/navigate_to_position')
        self._rotate_client = ActionClient(self, RotateAngle, f'{namespace}/rotate_angle')
        self._party_client = ActionClient(self, LedAnimation, f'{namespace}/led_animation')
        self.callback = callback
        self.bumps = self.create_subscription(HazardDetectionVector, f"{namespace}/hazard_detection", self.bump_callback, qos_profile_sensor_data)
        self.running = False
        self.avoid_angle = math.pi * 2
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
    
    def send_spin_goal(self, goal_heading, radians_per_sec=1.0):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = goal_heading
        goal_msg.max_rotation_speed = radians_per_sec
        self._rotate_client.wait_for_server()
        future = self._rotate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def send_party_goal(self):
      goal_msg = LedAnimation.Goal()
  
      goal_msg.animation_type = LedAnimation.Goal.SPIN_LIGHTS
      goal_msg.max_runtime.sec = 5  
      lightring_leds = LightringLeds()
  
      lightring_leds.leds = [
          LedColor(red=255, green=0, blue=0),    
          LedColor(red=0, green=255, blue=0),    
          LedColor(red=0, green=0, blue=255),    
          LedColor(red=255, green=0, blue=255),  
          LedColor(red=0, green=255, blue=255),  
          LedColor(red=255, green=255, blue=255) 
      ]
  
      lightring_leds.override_system = True
  
      goal_msg.lightring = lightring_leds
  
      self._party_client.wait_for_server()
      future = self._party_client.send_goal_async(goal_msg)
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
    state.spun = {}
    state.partied = {}
    
    planner = Planner.Planner()
    planner.declare_operators(Planner.move, Planner.spin, Planner.party)
    planner.declare_methods(Planner.navigate, Planner.spin_at_location, Planner.party_at_location)
    all_plans = (planner.anyhop(state, [('party_at_location','robot1', 'A'), ('spin_at_location', 'robot1', 'D'), ('party_at_location', 'robot1', 'F')]))S
    for plan in all_plans:
        if int(plan[1]) < chosen_plan_steps:
            chosen_plan_steps = plan[1]
            chosen_plan = plan


    global finish_flag
    rclpy.init(args=args)
    action_client = NavToActionClient(example_callback, namespace)
    action_client.spin_thread()
    for task in chosen_plan[0]:
        goal = task[-1]
        if goal == 'A':
            #316
            if task[0] == 'move':
                action_client.send_goal(False, [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
                input("Hit Enter Upon Move Completion \n")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion \n")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")
        elif goal == 'B':
            #Dr. Ferrer's Office Door
            if task[0] == 'move':
                action_client.send_goal(False, [[7.715,0.0,0.0], [0.0,0.0,0.0,1.0]])
                input("Hit Enter Upon Task Completion")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")

        elif goal == 'C':
            #Center of Long Hallway
            if task[0] == 'move':
                action_client.send_goal(False, [[7.715,10.6,0.0], [0.0,0.0,0.0,1.0]])
                # Unmeasured as of this time, to be checked
                input("Hit Enter Upon Task Completion")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")

        elif goal == 'D':
            #Lounge
            if task[0] == 'move':
                action_client.send_goal(False, [[3.215,10.6,0.0], [0.0,0.0,0.0,1.0]])
                #Unmeasured as of this time, to be checked
                input("Hit Enter Upon Task Completion")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")

        elif goal == 'E':
            #Dr. Goadrich's Office Door
            if task[0] == 'move':
                action_client.send_goal(False, [[7.715,21.445,0.0], [0.0,0.0,0.0,1.0]])
                input("Hit Enter Upon Task Completion")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")

        elif goal == 'F':
            #309 (I think that's the right room number)
            if task[0] == 'move':
                action_client.send_goal(False, [[1.515, 21.445,0.0], [0.0,0.0,0.0,1.0]])
                input("Hit Enter Upon Task Completion")
            elif task[0] == 'spin':
                action_client.send_spin_goal(math.pi * 2)
                input ("Hit Enter Upon Spin Completion")
            elif task[0] == 'party':
                action_client.send_party_goal()
                input("Hit Enter after the party! \n")
                
    rclpy.shutdown()
    print("done")

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()
