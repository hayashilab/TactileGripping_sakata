import serial
import time

# Replace with your ESP32 serial port
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200

class GripperController:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # wait for ESP32 reset
        print("Connected to gripper.")

    def send_command(self, command):
        """Send a string command to the ESP32."""
        cmd_str = f"{command}\n"
        self.ser.write(cmd_str.encode('utf-8'))
        print(f"Sent: {command}")

    def open(self):
        self.send_command("open")

    def close(self):
        self.send_command("close")

    def home(self):
        self.send_command("home")

    def close_serial(self):
        self.ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    gripper = GripperController()

    try:
        # Example sequence
        gripper.home()   # set current position as home
        time.sleep(1)
        gripper.open()   # open gripper
        time.sleep(8)
        gripper.close()  # close gripper
        time.sleep(8)
        gripper.open()   # open again
        time.sleep(8)
        gripper.close()  # back to home

    finally:
        gripper.close_serial()
