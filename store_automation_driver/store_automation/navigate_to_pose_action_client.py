import rclpy
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, Float32, Empty, UInt16, Bool
from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

ROS_ACTION_RESULT_SUCCESS = 4 # According to https://docs.ros.org/en/api/actionlib/html/index.html

class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        
        self.create_subscription(
            PoseStamped, '/call_navigate_to_pose', self._call_navigate_to_pose_callback, 1)
        
        self.create_subscription(
            Bool, '/cancel_goal', self._cancel_goal_callback, 1)
        
        self._feedback_publisher = self.create_publisher(Float32, '/navigate_to_pose_feedback', 1)
        self._result_publisher = self.create_publisher(Bool, '/navigate_to_pose_result', 1)
        
    def _call_navigate_to_pose_callback(self, msg):
        self.get_logger().info(f'goal callback: {msg}')
        self._send_goal(msg)
        
    def _cancel_goal_callback(self, msg):
        if self._goal_handle is not None:
            self._action_client._cancel_goal_async(self._goal_handle)

    def _send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self._result_publisher.publish(Bool(False))
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        action_result = True
        if result.status != ROS_ACTION_RESULT_SUCCESS:
            action_result = False
        self.get_logger().info(f'Result: {action_result}, code: {result.status}')
        msg = Bool()
        # msg.data = result.result
        msg.data = action_result
        self._result_publisher.publish(msg)
        
    def feedback_callback(self, feedback_msg):
        pass
        # feedback = feedback_msg.feedback
        # self.get_logger().info('Received feedback: {0}'.format(feedback.progress))
    
    
def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseActionClient()

    # action_client.send_goal('load_left')

    rclpy.spin(action_client)
    
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()