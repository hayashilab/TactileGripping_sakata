#!/usr/bin/env python3

# common packages
import os
import sys
from typing import Dict

# ros
import rclpy
from rclpy.node import Node

# msg
from std_msgs.msg import Int32

# srv
from std_srvs.srv import Empty

class GripperCommander(Node):
    def __init__(self, ros_params: Dict = {}):
        super().__init__('gripper_commander_node')
        
        self.ros_params = ros_params

        # publishers
        self.pose_in_step_pub = self.create_publisher(
            Int32, 
            "gripper/gripper_pose_in_step", 
            10
        )
        self.pose_in_mm_pub = self.create_publisher(
            Int32, 
            "gripper/gripper_pose_in_mm", 
            10
        )

        self.pose_in_mm = Int32()
        self.pose_in_step = Int32()

        self.gripper_mm_range = [47, 85] 
        self.gripper_mm_close_value = self.gripper_mm_range[0]
        self.gripper_mm_open_value = self.gripper_mm_range[-1]
        self.gripper_mm_standby_value = 45

        self.gripper_step_range = [400, 2257]
        self.gripper_step_close_value = self.gripper_step_range[-1]
        self.gripper_step_open_value = self.gripper_step_range[0]

        self.get_logger().info("GRIPPER COMMANDER INIT")

    def grasp_tomato(self, diameter=25):
        self.get_logger().info("grasp tomato")
        self.position_in_mm(diameter)
        
    def position_in_mm(self, pose_value):
        unit_type = "mm"
        value = self.check_gripper_range(pose_value, unit_type=unit_type)
        self.pose_in_mm.data = value

        self.pose_in_mm_pub.publish(self.pose_in_mm)
        self.waitGripper(pose=self.pose_in_mm, unit_type=unit_type, time=0.5)

    def position_in_step(self, step_value):
        unit_type = "step"
        value = self.check_gripper_range(step_value, unit_type=unit_type)
        self.pose_in_step.data = value

        self.pose_in_step_pub.publish(self.pose_in_step)
        self.waitGripper(pose=self.pose_in_step, unit_type=unit_type, time=0.5)

    def open(self):
        self.get_logger().info("open gripper")
        self.position_in_step(self.gripper_step_open_value)

    def close(self):
        self.get_logger().info("close gripper")
        self.position_in_step(self.gripper_step_close_value)

    def standby(self):
        self.get_logger().info("gripper standby")
        self.position_in_mm(self.gripper_mm_standby_value)
    
    def check_gripper_range(self, value, unit_type):
        if unit_type == "step":
            if value > self.gripper_step_range[-1]:
                self.get_logger().info(f"out of max range, max range is {self.gripper_step_range[-1]}")
                value = self.gripper_step_range[-1]
            elif value < self.gripper_step_range[0]:
                self.get_logger().info(f"out of min range, min range is {self.gripper_step_range[0]}")
                value = self.gripper_step_range[0]
        elif unit_type == "mm":
            if value > self.gripper_mm_range[-1]:
                self.get_logger().info(f"out of max range, max range is {self.gripper_mm_range[-1]}")
                value = self.gripper_mm_range[-1]
            elif value < self.gripper_mm_range[0]:
                self.get_logger().info(f"out of min range, min range is {self.gripper_mm_range[0]}")
                value = self.gripper_mm_range[0]
        else:
            self.get_logger().error("gripper range value's unit_type is not correctly set")
            return None
        
        return value

    def waitGripper(self, pose, unit_type, time=1.0):
        # Create a timer for 50ms (20Hz)
        iterations = int(20 * time)
        count = 0
        
        def timer_callback():
            nonlocal count
            if unit_type == "mm":
                self.pose_in_mm_pub.publish(pose)
            elif unit_type == "step":
                self.pose_in_step_pub.publish(pose)
            
            count += 1
            if count >= iterations:
                timer.cancel()
        
        timer = self.create_timer(0.05, timer_callback)  # 50ms = 20Hz

    def init_home(self):
        self.get_logger().info("init gripper home")
        
        client = self.create_client(Empty, "gripper/set_zero")
        
        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        
        # Create and send request
        request = Empty.Request()
        future = client.call_async(request)
        
        # Wait for future to complete
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info('gripper homing finished')

    def process(self):
        try:
            while rclpy.ok():
                message = """
                type e to exit program
                type h to homing
                type int value to control stepper motor in mm
                """
                x = input(message + "\n" + "enter: ")

                if x == "h":
                    self.get_logger().info("homing...")
                    self.init_home()
                elif x == "e":
                    self.get_logger().info("exiting program...")
                    sys.exit()
                elif x == "o":
                    self.open()
                elif x == "c":
                    self.close()
                elif x == "s":
                    self.standby()
                else:
                    try:
                        self.position_in_mm(int(x))
                    except ValueError:
                        self.get_logger().error(f"Invalid input: {x}")
                
                # Process any pending callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt, shutting down...")


def main(args=None):
    rclpy.init(args=args)
    
    gc = GripperCommander()
    
    try:
        gc.process()
    except Exception as e:
        gc.get_logger().error(f"Error: {e}")
    finally:
        gc.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()