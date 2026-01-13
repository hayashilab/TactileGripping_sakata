import serial, time

PORT = "/dev/ttyUSB_Gripper"
BAUD = 9600
ID   = 0xE0

def send(cmd_bytes, read_n=16):
    ser.write(bytes(cmd_bytes))
    ser.flush()
    time.sleep(0.05)
    rx = ser.read(read_n)
    print("tx:", bytes(cmd_bytes).hex(" "))
    print("rx_len:", len(rx), " rx:", rx.hex(" "))

ser = serial.Serial(PORT, BAUD, timeout=0.3)
ser.reset_input_buffer()

# 1) Read En pin status: E0 3A 1A  (Status: 01 enable / 02 disable / 00 error)
send([ID, 0x3A, (ID + 0x3A) & 0xFF])

# 2) Read shaft status: E0 3E 1E (01 blocked / 02 unblocked / 00 error)
send([ID, 0x3E, (ID + 0x3E) & 0xFF])

# 3) Read protection state (manual/wiki): E0 3E 1E 系と別に、保護解除は E0 3D 1D を使う
# Release locked-rotor protection: E0 3D 1D
send([ID, 0x3D, (ID + 0x3D) & 0xFF])

ser.close()
