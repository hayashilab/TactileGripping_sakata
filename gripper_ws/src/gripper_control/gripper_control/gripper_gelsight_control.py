#!/usr/bin/env python3
# gripper_control.py
import time
import threading
import rclpy
from rclpy.node import Node
from .motor_control import STEP_CONTROL # Importing a class called STEP_CONTROL from a file named motor_control.py in the same folder (likely this is your low-level motor interface).
from std_msgs.msg import Int32
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
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

        ###
        # 10s jog service
        self.jog_srv = self.create_service(Trigger, 'gripper/jog_10s', self.jog10sCallback)

        self._jog_lock = threading.Lock()
        self._jog_active = False
        self.jog_duration_sec = 10.0

        # 「回し続ける」ために非常に大きいstepを投げて、10秒後にstopMotorする
        self.jog_step = 0x7FFFFFFF  # 2,147,483,647（4バイト範囲内）
        self.jog_rpm = self.speed   # 既存のspeedを使用
        self.jog_dir = -1          # -1: CLOSE方向 / +1: OPEN方向（必要に応じて変更）
        ###

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
    
    ############################# 10s JOG FUNCTION #####################################
    def jog10sCallback(self, request, response):
        """Run motor for 10 seconds then stop (non-blocking service)."""
        with self._jog_lock:
            if self._jog_active:
                response.success = False
                response.message = "Jog already running"
                return response
            self._jog_active = True

        th = threading.Thread(target=self._jog10s_worker, daemon=True)
        th.start()

        response.success = True
        response.message = f"Jog started for {self.jog_duration_sec:.1f}s"
        return response

    def _jog10s_worker(self):
        try:
            # 念のためEnable（常時Enable運用なら不要ですが安全側に倒します）
            en_ret = self.step_control.setEnablePin(True)
            if en_ret != 1:
                self.get_logger().error(f"Failed to enable motor (ret={en_ret})")
                return

            step = int(self.jog_dir * self.jog_step)
            self.get_logger().info(f"Jog motor for {self.jog_duration_sec:.1f}s (rpm={self.jog_rpm}, step={step})")

            ret = self.step_control.setMotorRelativePose(self.jog_rpm, step)
            if ret != 1:
                self.get_logger().error(f"Failed to start jog (ret={ret})")
                return

            t_end = time.time() + self.jog_duration_sec
            while rclpy.ok() and time.time() < t_end:
                time.sleep(0.05)

        except Exception as e:
            self.get_logger().error(f"Jog worker exception: {e}")
        finally:
            # 10秒経過 or 異常時に停止
            stop_ret = self.step_control.stopMotor()
            if stop_ret != 1:
                self.get_logger().warn(f"stopMotor returned {stop_ret}")
            else:
                self.get_logger().info("Jog finished: motor stopped")

            with self._jog_lock:
                self._jog_active = False
    ###

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