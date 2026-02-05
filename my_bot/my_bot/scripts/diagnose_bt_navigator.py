#!/usr/bin/env python3
"""
Diagnose why bt_navigator won't activate
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import sys

class BTNavigatorDiagnostic(Node):
    def __init__(self):
        super().__init__('bt_navigator_diagnostic')
        
    def diagnose(self):
        """Check bt_navigator state and try to activate"""
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
            
        # Try to activate
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
                self.get_logger().error(f'✗ Activation failed: {result}')
        else:
            self.get_logger().error('✗ Activation call timed out')
            
        # Check final state
        request = GetState.Request()
        future = get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            state = future.result().current_state
            self.get_logger().info(f'Final state: {state.label} [{state.id}]')

def main(args=None):
    rclpy.init(args=args)
    diagnostic = BTNavigatorDiagnostic()
    diagnostic.diagnose()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
