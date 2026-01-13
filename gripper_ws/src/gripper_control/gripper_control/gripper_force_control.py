#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial, time
from .motor_control import STEP_CONTROL
from custom_msgs.msg import ReadySignal   # ✅ Add this import at the top
from std_msgs.msg import Int32, Bool
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

GRIPPER_OPEN_STEP  = -10000
GRIPPER_CLOSE_STEP = -35000
STEP_SIZE = 400



class GripperForceControl(Node):
    def __init__(self):
        super().__init__('gripper_force_control')
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)
        self.ser.reset_input_buffer()

        self.motor = STEP_CONTROL(com_port="/dev/ttyUSB_Gripper", baudrate=9600)
        self.motor.open()
        self.motor.setEnablePin(True)

        self.baseline = None
        self.threshold_drop = 15
        self.drop_count_required = 2

        qos_ready = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub_restart = self.create_subscription(
            Bool,
            '/drl_restart',
            self.on_restart,
            qos_ready
        )

        self.current_seq = 0

        self.sub_seq = self.create_subscription(
            ReadySignal,
            '/grasp_execute_ready',
            self.on_grasp_start,
            qos_ready
        )

        self.pub_grasp_result = self.create_publisher(
            ReadySignal,
            '/grasp_result',
            qos_ready
        )

        # Existing ROS2 service
        self.srv = self.create_service(Trigger, 'gripper_close_with_force', self.handle_close)
        self.get_logger().info("🟢 Ready: /gripper_close_with_force service.")


    def read_forces(self):
        """Read latest 3 values from Arduino"""
        line = ""
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
        try:
            vals = [int(x) for x in line.split(',')]
            if len(vals) == 3:
                return vals
        except Exception:
            pass
        return [999, 999, 999]

    def on_restart(self, msg: Bool):
        if msg.data:   # True = new cycle
            self.current_seq = 0
            self.baseline = None
            self.get_logger().info("🔄 New cycle detected → seq reset, baseline cleared.")

    def on_grasp_start(self, msg: ReadySignal):
        if msg.stage_name != "GRASP":
            return
        self.current_seq = msg.seq
        self.get_logger().info(f"🔢 Gripper received GRASP seq={self.current_seq}")


    def handle_close(self, req, res):
        """Force-controlled close sequence with auto reopen on failure"""

        # ---------------------------------------------------------------
        # Step 1 : Baseline sampling
        # ---------------------------------------------------------------
        samples = []
        for _ in range(5):
            vals = self.read_forces()
            if vals != [999, 999, 999]:
                samples.append(vals)
            time.sleep(0.1)

        if not samples:
            self.get_logger().error("❌ No valid force samples for baseline; aborting.")
            res.success = False
            res.message = "Failed baseline read"
            return res

        self.baseline = [sum(x[i] for x in samples) // len(samples) for i in range(3)]
        self.get_logger().info(f"📊 Baseline: {self.baseline}")

        # ---------------------------------------------------------------
        # Step 2 : Iterative close motion
        # ---------------------------------------------------------------
        rpm = 300
        success = False

        current_target = GRIPPER_OPEN_STEP   # -1000

        total_range = abs(GRIPPER_CLOSE_STEP - GRIPPER_OPEN_STEP)  # 34000

        for _ in range(0, total_range, STEP_SIZE):
            current_target -= STEP_SIZE

            # clamp inside safe range
            if current_target < GRIPPER_CLOSE_STEP:
                current_target = GRIPPER_CLOSE_STEP

            self.motor.setMotorAbsolutePose(rpm, current_target)
            time.sleep(0.05)

            current = self.read_forces()
            if current == [999, 999, 999]:
                continue

            drops = [b - c for b, c in zip(self.baseline, current)]
            self.get_logger().info(f"Force={current} Drops={drops}")

            # # Stop condition: ANY finger drop > threshold
            # if any(d > 4 for d in drops):
            #     self.get_logger().info("🟢 Drop detected → holding motor at current position (NO open).")

            #     # Hold current position
            #     self.motor.setEnablePin(True)

            #     # Ensure driver is READY for next move
            #     time.sleep(0.05)
            #     self.motor.clearAlarm() if hasattr(self.motor, 'clearAlarm') else None
            #     self.motor.setEnablePin(True)

            #     success = True
            #     break

            # Stop condition: ANY 2 fingers drop > 4
            if sum(d > 3 for d in drops) >= 2:
                self.get_logger().info("🟢 2-finger drop detected → holding motor at current position (NO open).")

                # Hold current position
                self.motor.setEnablePin(True)

                # Ensure driver is READY for next move
                time.sleep(0.05)
                if hasattr(self.motor, 'clearAlarm'):
                    self.motor.clearAlarm()
                self.motor.setEnablePin(True)

                success = True
                break

            # # Require ALL 3 fingers to drop by more than 5
            # if all(d > 5 for d in drops):
            #     self.get_logger().info("🟢 ALL fingers dropped → holding motor at current position (NO open).")

            #     # Hold current position
            #     self.motor.setEnablePin(True)

            #     time.sleep(0.05)
            #     if hasattr(self.motor, 'clearAlarm'):
            #         self.motor.clearAlarm()
            #     self.motor.setEnablePin(True)

            #     success = True
            #     break



            if current_target == GRIPPER_CLOSE_STEP:
                break

        # ---------------------------------------------------------------
        # Step 3 : Fallback full close (only if no drop detected)
        # ---------------------------------------------------------------
        if not success:
            self.get_logger().warn(
                "⚠️ No drop detected → closing fully to GRIPPER_CLOSE_STEP (-35000)."
            )
            self.motor.setMotorAbsolutePose(rpm, GRIPPER_CLOSE_STEP)
            time.sleep(1.0)
            self.motor.setEnablePin(True) 

        # ---------------------------------------------------------------
        # Step 4 : Publish grasp result
        # ---------------------------------------------------------------
        msg = ReadySignal()
        msg.seq = self.current_seq                        # ← echo seq from MotionPlanning
        msg.stage_name = "GRASP_RESULT"
        msg.ready = success
        msg.stamp = self.get_clock().now().to_msg()


        for _ in range(3):
            self.pub_grasp_result.publish(msg)
            time.sleep(0.1)

        if success:
            self.get_logger().info("📡 Published /grasp_result → SUCCESS ✅")
        else:
            self.get_logger().warn("📡 Published /grasp_result → FAIL ❌")

        # ---------------------------------------------------------------
        # Step 5 : Auto reopen only if failed
        # ---------------------------------------------------------------
        if not success:
            self.get_logger().info("🔁 Auto-opening gripper after failed grasp...")
            rpm_open = 200
            self.motor.setMotorAbsolutePose(rpm_open, GRIPPER_OPEN_STEP)
            time.sleep(4.0)
            self.motor.setEnablePin(True)   # ✅ IMPORTANT for next open cycle


        # ---------------------------------------------------------------
        # Step 6 : Set result
        res.success = success
        res.message = "Grasp success" if success else "Grasp failed"

        # 🟢 IMPORTANT: Re-enable after success so motor accepts next command
        if success:
            self.motor.setEnablePin(True)     # activate torque
            time.sleep(0.05)
            self.motor.setEnablePin(True)     # fully ready for next open command

        return res





def main(args=None):
    rclpy.init(args=args)
    node = GripperForceControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()