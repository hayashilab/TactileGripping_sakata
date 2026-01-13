#!/usr/bin/env python3
# gripper_utils.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, UInt8, Float32, Bool
from std_srvs.srv import Empty
import numpy as np
from tqdm import tqdm
# common
from scipy.interpolate import InterpolatedUnivariateSpline

class GRIPPER_UTILS:
    def __init__(self, node):
        # Store the node reference for creating publishers/subscribers
        self.node = node
        
        # Initialize publisher
        self.gripper_pub = self.node.create_publisher(
            Float64, 
            '/gripper/position_in_mm_sub', 
            10
        )
        
        self.pose = Float64()
        
        # Interpolation values
        x = [-11, 5, 10, 30, 78, 110, 134, 141]  # actual value measure
        y = [100, 93, 90, 80, 60, 40, 20, 0]     # value to send to stepper motor
        self.mm2step_gripper = InterpolatedUnivariateSpline(x, y, k=5, check_finite=False)

    def setGripper(self, _pose):
        """Set gripper position in mm, converted to steps"""
        self.node.get_logger().info(f"step in mm actual : {self.mm2step_gripper(_pose)}")
        self.pose.data = int(self.mm2step_gripper(_pose))
        self.gripper_pub.publish(self.pose)

    def initHome(self):
        """Initialize gripper to home position using service call"""
        self.node.get_logger().info("init gripper home")
        
        # Create client
        client = self.node.create_client(Empty, '/gripper/init_home')
        
        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting...')
        
        # Call service
        request = Empty.Request()
        future = client.call_async(request)
        
        # Wait for service response
        rclpy.spin_until_future_complete(self.node, future)
        
        self.node.get_logger().info('gripper homing finished')
        return future.result()

    def setGripperRaw(self, _pose):
        """Set gripper position using raw steps value"""
        self.pose.data = int(_pose)
        self.gripper_pub.publish(self.pose)

    def waitGripper(self, time=1.0):
        """Wait for gripper movement to complete"""
        total_iterations = int(10 * time)
        
        for _ in tqdm(range(total_iterations)):
            self.gripper_pub.publish(self.pose)
            # Use ROS2 sleep
            self.node.create_rate(10).sleep()