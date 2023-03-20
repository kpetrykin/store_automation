import rclpy
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, Float32

from store_automation_actions.action import ControlArms

class ArmControlActionClient(Node):

    def __init__(self):
        super().__init__('arm_control_action_client')
        self._action_client = ActionClient(self, ControlArms, 'control_arms')
        
        self.create_subscription(
            String, '/call_arm_operation', self._call_arm_operation_callback, 1)
        
        self._feedback_publisher = self.create_publisher(Float32, '/arm_operation_feedback', 1)
        self._result_publisher = self.create_publisher(String, '/arm_operation_result', 1)
        
    def _call_arm_operation_callback(self, msg):
        self.get_logger().info('goal callback')
        self._send_goal(msg.data)

    def _send_goal(self, operation):
        goal_msg = ControlArms.Goal()
        goal_msg.operation = operation

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self._result_publisher.publish(String(str(False)))
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        msg = String()
        msg.data = str(result.result)
        self._result_publisher.publish(msg)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.progress))
    
    
def main(args=None):
    rclpy.init(args=args)

    action_client = ArmControlActionClient()

    # action_client.send_goal('load_left')

    rclpy.spin(action_client)
    
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()