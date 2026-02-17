#!/usr/bin/env python3
# gripper_control.py


import rclpy
from rclpy.node import Node
from .motor_control import STEP_CONTROL # Importing a class called STEP_CONTROL from a file named motor_control.py in the same folder (likely this is your low-level motor interface).
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty
from std_srvs.srv import Trigger, SetBool
# common
from scipy.interpolate import InterpolatedUnivariateSpline # This helps convert distances in millimeters to motor steps using interpolation.

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        
        # Create subscriptions
        self.subscription_step = self.create_subscription(
            Int32,
            'gripper/gripper_pose_in_step',
            self.gripperPoseInStepCallback,
            10
        )
        
        self.subscription_mm = self.create_subscription(
            Int32,
            'gripper/gripper_pose_in_mm',
            self.gripperPoseInMMCallback,
            10
        )
        
        # Create service
        self.srv = self.create_service(
            Empty,
            'gripper/set_zero',
            self.setZeroCallBack
        )
        
        self.encoder_srv = self.create_service(Trigger, 'gripper/read_encoder', self.readEncoderCallback)

        # ---- Auto control service (contact-based) ----
        self.auto_control_srv = self.create_service(
            SetBool,
            'gripper/auto_control',
            self.autoControlCallback
        )
        self._auto_control_enabled = False
        self._contact_state = "INITIALIZING"

        # Subscribe to contact state from tactile node
        self.subscription_contact = self.create_subscription(
            String,
            '/tactile/contact_state',
            self.contactStateCallback,
            10
        )

        # Publisher for current gripper step (for tactile node logging)
        self.pub_current_step = self.create_publisher(Int32, '/gripper/current_step', 10)

        # Auto control parameters
        self._auto_step_increment = 500  # step increment per cycle when closing
        self._auto_close_limit = -41000  # maximum close position (steps)
        self._auto_open_position = -1000  # open position (steps)

        # Motor control settings
        self.speed = 1000
        self.previous_cmd_pose = -1000000 # previous_cmd_pose: remembers the last position (to avoid sending duplicates)
        self.cmd_timeout = 10
        self.com_port = "/dev/ttyUSB_Gripper" # USB serial port used to communicate with the gripper
        self.baudrate = 9600
        
        # Initialize motor control
        self.step_control = STEP_CONTROL(com_port=self.com_port, baudrate=self.baudrate)
        self.open()
        
        # Print motor status
        self.get_logger().info(f'encoder value: {self.step_control.readEncoderValue()}')
        self.get_logger().info(f'Pulse received: {self.step_control.readReceivedPulses()}')
        self.get_logger().info(f'Motor angle: {self.step_control.readMotorShaftAngle()}')
        self.get_logger().info(f'error: {self.step_control.readMotorShaftErrorAngle()}')
        self.get_logger().info(f'en pin: {self.step_control.readEnPinStatus()}')
        self.get_logger().info(f'shaft status: {self.step_control.readShaftStatus()}')



############################ MANUAL CALLIBRATION ###################################


        # mm increasing (SciPy requirement)
        x = [23, 34, 46, 51, 56, 63, 75, 80, 90, 100]

        # steps corrected: 23mm → -35000 (CLOSE), 100mm → -1000 (OPEN)
        y = [-35000, -32000, -30000, -25000, -20000, -15000, -10000, -7000, -4000, -1000]

     
        self.mm2step_gripper = InterpolatedUnivariateSpline(x, y, k=5, check_finite=False)

        # Timer for auto control (10Hz)
        self._auto_control_timer = self.create_timer(0.1, self._auto_control_loop)

    def contactStateCallback(self, msg: String):
        """Callback for contact state from tactile node"""
        self._contact_state = msg.data

    def autoControlCallback(self, request, response):
        """Enable/disable auto control based on contact state"""
        if request.data:
            # Enable auto control
            self._auto_control_enabled = True
            # Start from open position
            self.previous_cmd_pose = self._auto_open_position
            response.success = True
            response.message = "Auto control enabled. Gripper will close until contact."
            self.get_logger().info("Auto control ENABLED")
        else:
            # Disable auto control
            self._auto_control_enabled = False
            response.success = True
            response.message = "Auto control disabled."
            self.get_logger().info("Auto control DISABLED")
        return response

    def _auto_control_loop(self):
        """Auto control loop: close on NON_CONTACT, stop on CONTACT"""
        # Publish current step for tactile node
        step_msg = Int32()
        step_msg.data = self.previous_cmd_pose
        self.pub_current_step.publish(step_msg)

        if not self._auto_control_enabled:
            return

        if self._contact_state == "INITIALIZING":
            # Wait for detector to initialize
            return

        if self._contact_state == "NON_CONTACT":
            # Close gripper incrementally
            new_step = self.previous_cmd_pose - self._auto_step_increment
            new_step = max(new_step, self._auto_close_limit)  # Don't exceed limit

            if new_step != self.previous_cmd_pose:
                pose_msg = Int32()
                pose_msg.data = new_step
                self.gripperPoseInStepCallback(pose_msg)
                self.get_logger().info(f"Auto: Closing to {new_step}")

        elif self._contact_state == "CONTACT":
            # Stop - do nothing, gripper stays at current position
            self.get_logger().info("Auto: CONTACT detected - stopping", throttle_duration_sec=1.0)

    def readEncoderCallback(self, request, response):
        value, ret = self.step_control.readEncoderValue()
        if ret == 1:
            response.success = True
            response.message = str(value)
        else:
            response.success = False
            response.message = "Failed to read encoder"
        return response
        
    def open(self):
        """Open serial connection to motor controller"""
        ret = self.step_control.open()
        if not ret:
            self.get_logger().error("Com Port Error")
            exit()
        self.get_logger().info(f'En pin: {self.step_control.setEnablePin(True)}')
        
    def gripperPoseInMMCallback(self, data):
        """Callback for gripper position in millimeters"""
        data.data = min(data.data, 90)
        pose_in_step = Int32()
        pose_in_step.data = int(self.mm2step_gripper(data.data))
        pose_in_step.data = max(pose_in_step.data, -41000)
        self.gripperPoseInStepCallback(pose_in_step)
        
    def gripperPoseInStepCallback(self, data):
        """Callback for gripper position in steps"""
        self.get_logger().info(f'{data.data}')
        
        # Allow duplicate OPEN command (-1000 step)
        if data.data == self.previous_cmd_pose and data.data != -10000:
            return

            
        done = False
        count = 0
        while not done:
            self.get_logger().info(f"Set stepper at {data.data}")
            ret = self.step_control.setMotorAbsolutePose(self.speed, data.data)
            if ret == 1:
                done = True
            count += 1
            if count >= self.cmd_timeout:
                break
                
        self.previous_cmd_pose = data.data

        # # 🟢 Read and print actual encoder feedback after move
        # raw_value, ret = self.step_control.readEncoderValue()
        # if ret == 1:
        #     self.get_logger().info(f"👉 Actual encoder after move = {raw_value}")
        # else:
        #     self.get_logger().warn("❌ Failed to read encoder after move")
        
    def setZeroCallBack(self, request, response):
        """Service callback to set zero position"""
        self.get_logger().info(f"Set Zero: {self.step_control.setZero(50, 50)}") # if wanna reserve set zero rotation, put 50, 50
        self.get_logger().info(f'encoder value: {self.step_control.readEncoderValue()}')
        self.previous_cmd_pose = 0
        return response
        
    def process(self):
        """Main processing loop"""
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.step_control.close()
            self.get_logger().info("Serial connection closed")


def main(args=None):
    rclpy.init(args=args)
    
    gripper_control = GripperControl()
    gripper_control.process()
    
    # Cleanup
    gripper_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()