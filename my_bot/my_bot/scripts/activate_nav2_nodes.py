#!/usr/bin/env python3
"""
Script to manually activate Nav2 lifecycle nodes
This works around the lifecycle manager timing issues
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
import time

class Nav2Activator(Node):
    def __init__(self):
        super().__init__('nav2_activator')
        self.nodes = [
            'map_server',
            'amcl',
            'planner_server',
            'controller_server',
            'bt_navigator',
            'waypoint_follower',
            'smoother_server'
        ]
        
    def get_node_state(self, node_name):
        """Get current state of a lifecycle node"""
        client = self.create_client(GetState, f'/{node_name}/get_state')
        
        if not client.wait_for_service(timeout_sec=5.0):
            return None
            
        request = GetState.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is None:
            return None
            
        return future.result().current_state.id
        
    def change_state(self, node_name, transition_id, transition_label):
        """Change state of a lifecycle node"""
        client = self.create_client(ChangeState, f'/{node_name}/change_state')
        
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f'Service /{node_name}/change_state not available')
            return False
            
        request = ChangeState.Request()
        request.transition.id = transition_id
        request.transition.label = transition_label
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().warn(f'Failed to call {transition_label} on {node_name}')
            return False
            
        if not future.result().success:
            self.get_logger().warn(f'{transition_label} transition failed for {node_name}')
            return False
            
        return True
        
    def activate_node(self, node_name):
        """Activate a lifecycle node (configure if needed, then activate)"""
        # Check current state
        current_state = self.get_node_state(node_name)
        
        if current_state is None:
            self.get_logger().warn(f'Could not get state for {node_name}')
            return False
            
        # State IDs: 0=unconfigured, 1=inactive, 2=active, 3=shutdown
        if current_state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f'{node_name} is already active')
            return True
            
        # If unconfigured, configure first
        if current_state == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info(f'Configuring {node_name}...')
            if not self.change_state(node_name, Transition.TRANSITION_CONFIGURE, 'configure'):
                return False
            self.get_logger().info(f'✓ Configured {node_name}')
            time.sleep(1.5)  # Wait for configuration to complete
            
        # Now activate (if not already active)
        current_state = self.get_node_state(node_name)
        if current_state == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info(f'Activating {node_name}...')
            if not self.change_state(node_name, Transition.TRANSITION_ACTIVATE, 'activate'):
                return False
            self.get_logger().info(f'✓ Activated {node_name}')
            return True
        elif current_state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f'✓ {node_name} is already active')
            return True
        else:
            self.get_logger().warn(f'{node_name} is in unexpected state: {current_state}')
            return False
        
    def activate_all(self):
        """Activate all Nav2 nodes"""
        self.get_logger().info('Waiting for nodes to be ready...')
        time.sleep(3.0)  # Wait for all nodes to start
        
        for node_name in self.nodes:
            self.get_logger().info(f'Processing {node_name}...')
            if self.activate_node(node_name):
                self.get_logger().info(f'✓ {node_name} is active')
            else:
                self.get_logger().error(f'✗ Failed to activate {node_name}')
            time.sleep(0.5)
            
        self.get_logger().info('Done! Check node states with: ros2 lifecycle get /<node_name>')

def main(args=None):
    rclpy.init(args=args)
    activator = Nav2Activator()
    activator.activate_all()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
