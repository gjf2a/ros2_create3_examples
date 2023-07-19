import rclpy
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from rclpy.node import Node
import math, sys

class RotateActionClient(Node):
    def __init__(self, callback, namespace):
        super().__init__("Demo")
        self._action_client = ActionClient(self, RotateAngle, f'{namespace}/rotate_angle')
        self.callback = callback
        
    def send_goal(self):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = math.pi / 2
        goal_msg.max_rotation_speed = 0.5
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


def example_callback(future):
    result = future.result().result
    print("finished...", result)
    rclpy.shutdown()


def main(args=None, namespace=''):
    rclpy.init(args=args)

    action_client = RotateActionClient(example_callback, namespace)
    action_client.send_goal()
    rclpy.spin(action_client)

    #rclpy.spin_until_future_complete(action_client, future)

    #result = future.result()

    #if result.status == 0:
    #    print('Goal succeeded!')
    #else:
    #    print('Goal failed with status code:', result.status)

    #input("Enter a key")
    #action_client.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        main(namespace=f'/{sys.argv[1]}')
    else:
        main()
