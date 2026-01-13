#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

OPEN_ENCODER  = 18537
CLOSE_ENCODER = 43112

OPEN_THRESHOLD  = OPEN_ENCODER  + 1500
CLOSE_THRESHOLD = CLOSE_ENCODER - 1500

GRIPPER_OPEN_STEP  = -10000
GRIPPER_CLOSE_STEP = -35000


class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')

        # Publisher to mm topic (gripper_control.py listens here)
        self.pub_mm = self.create_publisher(Int32, 'gripper/gripper_pose_in_mm', 10)

        # Define open/close values (calibrated in mm)
        self.open_mm = 75   # fully open
        self.close_mm = 23   # fully close (max travel, will auto-stop via force sensors)

        # === Service clients ===
        self.encoder_client = self.create_client(Trigger, 'gripper/read_encoder')
        while not self.encoder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper/read_encoder service...')

        # 🟢 Client to force-based close service
        self.force_client = self.create_client(Trigger, 'gripper_close_with_force')
        while not self.force_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper_close_with_force service...')

        # === Create services for open/close ===
        self.srv_open = self.create_service(Trigger, '/open_gripper', self.open_callback)
        self.srv_close = self.create_service(Trigger, '/close_gripper', self.close_callback)
        self.get_logger().info("✅ Gripper services [/open_gripper, /close_gripper] are ready.")


    # --------------------------------------------------------------------------
    def call_encoder(self, callback=None):
        """Async call to encoder service"""
        req = Trigger.Request()
        future = self.encoder_client.call_async(req)

        def done_callback(fut):
            try:
                res = fut.result()
                if res.success:
                    val = int(res.message)
                    self.get_logger().info(f"Encoder response = {val}")
                    if callback:
                        callback(val)
                else:
                    self.get_logger().warn(f"Encoder failed: {res.message}")
                    if callback:
                        callback(None)
            except Exception as e:
                self.get_logger().error(f"Encoder call exception: {e}")
                if callback:
                    callback(None)

        future.add_done_callback(done_callback)

    # --------------------------------------------------------------------------
    def open_callback(self, request, response):
        """Open gripper immediately without checking encoder."""
        
        self.get_logger().info("🟢 /open_gripper called → sending OPEN command (no encoder check).")

        msg = Int32()
        msg.data = self.open_mm     # 100 mm → -1000 steps in gripper_control
        self.pub_mm.publish(msg)

        self.get_logger().info(f"🟢 Commanding OPEN: {self.open_mm} mm → -10000 step")

        response.success = True
        response.message = "Open command sent (no encoder check)"
        return response





    # --------------------------------------------------------------------------
    def close_callback(self, request, response):
        """Handle /close_gripper service (force-controlled close)"""
        self.get_logger().info("🟢 /close_gripper called → delegating to /gripper_close_with_force")

        req = Trigger.Request()
        future = self.force_client.call_async(req)

        def done_callback(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f"✅ Force-based close success: {res.message}")
                else:
                    self.get_logger().warn(f"❌ Force-based close failed: {res.message}")
            except Exception as e:
                self.get_logger().error(f"Force-based close exception: {e}")

        future.add_done_callback(done_callback)

        response.success = True
        response.message = "Force-based close triggered."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()