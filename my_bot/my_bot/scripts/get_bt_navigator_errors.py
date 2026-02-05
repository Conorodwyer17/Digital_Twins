#!/usr/bin/env python3
"""
Get detailed error information from bt_navigator
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableTransitions
from lifecycle_msgs.msg import Transition
import sys

class BTNavigatorErrorChecker(Node):
    def __init__(self):
        super().__init__('bt_navigator_error_checker')
        
    def check(self):
        node_name = 'bt_navigator'
        
        # Get current state
        get_state_client = self.create_client(GetState, f'/{node_name}/get_state')
        if not get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service /{node_name}/get_state not available')
            return
            
        request = GetState.Request()
        future = get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            state = future.result().current_state
            self.get_logger().info(f'Current state: {state.label} [{state.id}]')
        else:
            self.get_logger().error('Could not get state')
            return
            
        # Get available transitions
        get_transitions_client = self.create_client(GetAvailableTransitions, f'/{node_name}/get_available_transitions')
        if get_transitions_client.wait_for_service(timeout_sec=5.0):
            request = GetAvailableTransitions.Request()
            future = get_transitions_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result():
                transitions = future.result().available_transitions
                self.get_logger().info(f'Available transitions:')
                for trans in transitions:
                    self.get_logger().info(f'  - {trans.transition.label} [{trans.transition.id}]')
        
        # Try to activate and see what happens
        change_state_client = self.create_client(ChangeState, f'/{node_name}/change_state')
        if not change_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service /{node_name}/change_state not available')
            return
            
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        request.transition.label = 'activate'
        
        self.get_logger().info('Attempting to activate...')
        future = change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            result = future.result()
            if result.success:
                self.get_logger().info('✓ Activation successful!')
            else:
                self.get_logger().error(f'✗ Activation failed')
                self.get_logger().error(f'Response: {result}')
        else:
            self.get_logger().error('✗ Activation call timed out')
            
        # Check parameters
        self.get_logger().info('\nChecking bt_navigator parameters...')
        try:
            import subprocess
            result = subprocess.run(['ros2', 'param', 'get', '/bt_navigator', 'default_nav_to_pose_bt_xml'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(f'default_nav_to_pose_bt_xml: {result.stdout.strip()}')
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    checker = BTNavigatorErrorChecker()
    checker.check()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
