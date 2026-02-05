#!/usr/bin/env python3
"""
Simple script to publish an initial pose for AMCL
This allows Nav2 to start without requiring manual initial pose setting
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        # Wait a bit for AMCL to be ready
        time.sleep(3.0)
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        """Publish initial pose at origin (0, 0, 0)"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set pose to origin (you can adjust these values based on where your robot spawns)
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (uncertainty in the initial pose)
        # Position covariance
        msg.pose.covariance[0] = 0.25  # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.06853891909122467  # yaw
        
        self.publisher.publish(msg)
        self.get_logger().info('Published initial pose to /initialpose')
        self.get_logger().info('Initial pose: x=0.0, y=0.0, yaw=0.0')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    # Give it time to publish, then shutdown
    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
