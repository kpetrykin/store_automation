import rclpy
import time
import math
import threading
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

from store_automation_actions.action import ControlArms

OPERATION_LOAD_LEFT = 'load_left'
OPERATION_LOAD_RIGHT = 'load_right'
OPERATION_UNLOAD_LEFT = 'unload_left'
OPERATION_UNLOAD_RIGHT = 'unload_right'
OPERATION_STANDBY = 'standby'

PITCH_STANDBY = 0.001
PITCH_LOAD = 1.0
YAW_OPEN = 0.5
YAW_STANDBY = 0.0
SLIDER_PULLED_OUT = 0.5
SLIDER_PUSHED_IN = 0.001
GRIPPER_LOCKED = 0.5
GRIPPER_UNLOCKED = 0.001

OPERATION_EXEC_TIME_STEP = 0.5
OPERATION_EXEC_TIMEOUT = 200


class ArmControlActionServer(Node):
    def __init__(self) -> None:
        super().__init__('arm_control_action_server')
        
        # Multithreading 
        self._cb_group1 = MutuallyExclusiveCallbackGroup()
        self._cb_group2 = MutuallyExclusiveCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            ControlArms,
            'control_arms',
            self._execute_callback,
            callback_group=self._cb_group1)
        
        self._operations = [OPERATION_LOAD_LEFT, OPERATION_LOAD_RIGHT,
                            OPERATION_UNLOAD_LEFT, OPERATION_UNLOAD_RIGHT, OPERATION_STANDBY]
        
        self.create_subscription(
            Float32, '/cur_pitch_angle', self._cur_pitch_angle_callback, 1, callback_group=self._cb_group2)
        
        self._cur_pitch_lock = threading.Lock()
        self._cur_pitch = 0.0
        self._cur_pitch_epsilon = 0.05
        
        self._yaw_publisher = self.create_publisher(Float32, '/arm_yaw', 1)
        self._pitch_publisher = self.create_publisher(Float32, '/arm_pitch', 1)
        self._slider_publisher = self.create_publisher(Float32, '/arm_slider', 1)
        self._gripper_publisher = self.create_publisher(Float32, '/gripper_pos', 1)
        
        
    def _execute_callback(self, goal_handle) -> None:
        self.get_logger().info(f'Received goal {goal_handle.request.operation}...')
        
        operation_result = False
        
        if goal_handle.request.operation in self._operations:
            if goal_handle.request.operation == OPERATION_LOAD_LEFT:
                operation_result = self._exec_load('left', goal_handle)
            elif goal_handle.request.operation == OPERATION_LOAD_RIGHT:
                operation_result = self._exec_load('right', goal_handle)
            elif goal_handle.request.operation == OPERATION_UNLOAD_LEFT:
                operation_result = self._exec_unload('left', goal_handle)
            elif goal_handle.request.operation == OPERATION_UNLOAD_RIGHT:
                operation_result = self._exec_unload('right', goal_handle)
            elif goal_handle.request.operation == OPERATION_STANDBY:
                operation_result = self._exec_standby(goal_handle)
                
            if operation_result:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        else:
            goal_handle.abort()
            
        result = ControlArms.Result()
        
        result.result = operation_result
        
        return result
    
    def _cur_pitch_angle_callback(self, msg) -> None:
        with self._cur_pitch_lock:
            self._cur_pitch = msg.data
            # self.get_logger().info(f'Callback pitch {self._cur_pitch}')
            
    def _exec_load(self, direction: str, goal_handle) -> bool:
        feedback_msg = ControlArms.Feedback()
        feedback_msg.progress = 0.0
        
        dir_coeff = 1
        if direction == 'left':
            dir_coeff = -1
        
        self._pitch_publisher.publish(Float32(data=PITCH_LOAD * dir_coeff))
        self._yaw_publisher.publish(Float32(data=YAW_OPEN * dir_coeff))
        self._slider_publisher.publish(Float32(data=SLIDER_PULLED_OUT))
        self._gripper_publisher.publish(Float32(data=GRIPPER_UNLOCKED))
        goal_handle.publish_feedback(feedback_msg)
        
        if self._wait_pitch(PITCH_LOAD, goal_handle, feedback_msg) == False:
            return False
        
        self._gripper_publisher.publish(Float32(data=GRIPPER_LOCKED))
        self._yaw_publisher.publish(Float32(data=YAW_STANDBY))
        
        time.sleep(2.0)
        
        self._pitch_publisher.publish(Float32(data=PITCH_STANDBY * dir_coeff))
        if self._wait_pitch(PITCH_STANDBY, goal_handle, feedback_msg) == False:
            return False
        
        self._slider_publisher.publish(Float32(data=SLIDER_PUSHED_IN))
        
        return True
        
            
    def _wait_pitch(self, setpoint, goal_handle, feedback_msg) -> bool:
        with self._cur_pitch_lock:
            pitch = self._cur_pitch
            
        timeout_counter = 0
        
        while abs(pitch) < abs(setpoint) - self._cur_pitch_epsilon or abs(pitch) > abs(setpoint) + self._cur_pitch_epsilon:
            with self._cur_pitch_lock:
                pitch = self._cur_pitch
                
            timeout_counter += 1
            feedback_msg.progress = pitch
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(OPERATION_EXEC_TIME_STEP)
            if timeout_counter > OPERATION_EXEC_TIMEOUT:
                return False # Operation failure by timeout
    
    def _exec_unload(self, direction: str, goal_handle) -> bool:
        feedback_msg = ControlArms.Feedback()
        feedback_msg.progress = 0.0
        
        dir_coeff = 1
        if direction == 'left':
            dir_coeff = -1
        
        self._pitch_publisher.publish(Float32(data=PITCH_LOAD * dir_coeff))
        self._slider_publisher.publish(Float32(data=SLIDER_PULLED_OUT))
        goal_handle.publish_feedback(feedback_msg)
        
        if self._wait_pitch(PITCH_LOAD, goal_handle, feedback_msg) == False:
            return False
        
        self._gripper_publisher.publish(Float32(data=GRIPPER_UNLOCKED))
        self._yaw_publisher.publish(Float32(data=YAW_OPEN * dir_coeff))
                
        time.sleep(2.0)
        
        self._pitch_publisher.publish(Float32(data=PITCH_STANDBY * dir_coeff))
        if self._wait_pitch(PITCH_STANDBY, goal_handle, feedback_msg) == False:
            return False
        
        self._slider_publisher.publish(Float32(data=SLIDER_PUSHED_IN))
        self._gripper_publisher.publish(Float32(data=GRIPPER_LOCKED))
        self._yaw_publisher.publish(Float32(data=YAW_STANDBY))
        
        return True
    
    def _exec_standby(self, goal_handle) -> bool:
        feedback_msg = ControlArms.Feedback()
        feedback_msg.progress = 0.0
        
        self._slider_publisher.publish(Float32(data=SLIDER_PUSHED_IN))
        self._gripper_publisher.publish(Float32(data=GRIPPER_LOCKED))
        self._yaw_publisher.publish(Float32(data=YAW_STANDBY))
        
        self._pitch_publisher.publish(Float32(data=PITCH_STANDBY))
        if self._wait_pitch(PITCH_STANDBY, goal_handle, feedback_msg) == False:
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)

    try:
        acas = ArmControlActionServer()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(acas)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            acas.destroy_node()

    finally:
        # shutdown the ROS communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()