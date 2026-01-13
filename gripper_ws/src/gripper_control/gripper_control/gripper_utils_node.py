#!/usr/bin/env python3
# gripper_utils_node.py

import rclpy
from rclpy.node import Node
from .gripper_utils import GRIPPER_UTILS

class GripperUtilsNode(Node):
    def __init__(self):
        super().__init__('gripper_utils_node')
        
        # Initialize the gripper utils class with this node
        self.gripper_utils = GRIPPER_UTILS(self)
        
        # Log initialization
        self.get_logger().info('Gripper Utils Node initialized')
        
    def initialize_gripper(self):
        """Initialize the gripper to home position"""
        self.gripper_utils.initHome()
        
    def set_gripper_position(self, position_mm):
        """Set the gripper position in millimeters"""
        self.gripper_utils.setGripper(position_mm)
        

def main(args=None):
    rclpy.init(args=args)
    
    gripper_utils_node = GripperUtilsNode()
    
    try:
        rclpy.spin(gripper_utils_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        gripper_utils_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()