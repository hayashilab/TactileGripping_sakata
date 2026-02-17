# motor_control.py

import serial
import time


class STEP_CONTROL:
    def __init__(self, id = 0xe0, com_port = '/dev/ttyUSB_Gripper', baudrate = 9600, step_angle = 1.8, m_step = 16):
        self.id = id
        self.step_angle = step_angle
        self.m_step = m_step
        self.pulse_per_round = self.m_step * 360.0 / self.step_angle
        self.ser = serial.Serial()
        self.ser.baudrate = baudrate
        self.ser.port = com_port
        self.ser.timeout = 1

    def open(self):
        self.ser.open()
        if not self.ser.is_open:
            print('Can not open com port')
        return self.ser.is_open

    def close(self):
        self.ser.close()
        return not self.ser.is_open

    def chkSum(self, data):  # sourcery skip: avoid-builtin-shadow
        sum = 0x00
        for d in data:
            sum += d
        return sum & 0xff

    def write(self, data, tCHK):
        out = data
        out.append(tCHK)
        
        self.ser.write(bytearray(out))
    
    def read(self,no_byte):
        s = self.ser.read(no_byte)
        return list(s)

    def chkReturn(self, rx_byte, data):
        if len(data) != rx_byte:
            return -1
        if data[0] != self.id:
            return -2
        tCHK = self.chkSum(data[:(rx_byte-1)])
        if data[rx_byte-1] != tCHK:
            return -3
        return 1

    def retUint16(self,data):
        value = int.from_bytes(bytearray(data), byteorder='big',signed = False)
        return value

    def retInt16(self,data):
        value = int.from_bytes(bytearray(data), byteorder='big',signed = True)
        return value

    def retInt32(self,data):
        value = int.from_bytes(bytearray(data), byteorder='big',signed = True)
        return value

    def calSpeedCmd(self, rpm):
        speed_cmd = (rpm * self.m_step * (360.0 / self.step_angle)) / 30000.0
        speed_cmd = 1 if speed_cmd < 1 else int(speed_cmd)
        speed_cmd = 127 if speed_cmd > 127 else int(speed_cmd)
        return speed_cmd

    def calSpeedRPM(self, speed_cmd):
        rpm = (speed_cmd * 30000.0) / (self.m_step * (360.0 / self.step_angle))
        return rpm


    def readEncoderValue(self):
        rx_byte = 4
        cmd = [self.id,0x30]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        return self.retUint16(rx_data[1:rx_byte-1]), 1

    def readReceivedPulses(self):
        rx_byte = 6
        cmd = [self.id,0x33]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        return self.retInt32(rx_data[1:rx_byte-1]), 1

    def readMotorShaftAngle(self):
        rx_byte = 6
        cmd = [self.id,0x36]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        return float(self.retInt32(rx_data[1:rx_byte-1])) * 360.0 / 65535.0, 1

    def readMotorShaftErrorAngle(self):
        rx_byte = 4
        cmd = [self.id,0x39]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        return float(self.retInt32(rx_data[1:rx_byte-1])) * 360.0 / 65535.0, 1

    def readEnPinStatus(self):
        rx_byte = 3
        cmd = [self.id,0x3a]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        # 1 = enable, 2 = disable, 0 = error
        return rx_data[1], 1

    def readShaftStatus(self):
        rx_byte = 3
        cmd = [self.id,0x3e]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return (0,chk_ret)
        
        # 1 = blocked, 2 = unblocked, 0 = error
        return rx_data[1], 1

    def setEnablePin(self, en):
        rx_byte = 3
        cmd = [self.id, 0xf3, en]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return chk_ret

        if chk_ret == 1:
            return 1
        
        return 0

    def stopMotor(self):
        rx_byte = 3
        cmd = [self.id, 0xf7]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return chk_ret

        if chk_ret == 1:
            return 1
        
        return 0

    def setMotorRelativePose(self,rpm,step):
        # sourcery skip: avoid-builtin-shadow
        rx_byte = 3
        dir = 1 << 7 if step >= 0 else 0
        speed = self.calSpeedCmd(rpm)
        third_byte = dir | speed
        step = abs(step)
        step = step.to_bytes(4,'big')

        cmd = [self.id, 0xfd, third_byte, step[0], step[1], step[2], step[3]]

        tCHK = self.chkSum(cmd)
        self.write(cmd,tCHK)
        rx_data = self.read(rx_byte)

        chk_ret = self.chkReturn(rx_byte, rx_data)
        if (self.chkReturn(rx_byte, rx_data) < 0):
            return chk_ret

        if chk_ret == 1:
            return 1

        return 0

    def setMotorAbsolutePose(self,rpm,pose):
        pulse, ret = self.readReceivedPulses()
        if ret != 1:
            return ret

        delta_pulse = pose - pulse
        #print(delta_pulse)
        return self.setMotorRelativePose(rpm,delta_pulse)

    def calDelayTime(self,rpm,step):
        real_rpm = self.calSpeedRPM(self.calSpeedCmd(rpm))
        return (float(abs(step)) / float(self.pulse_per_round) / float(real_rpm)) * 60.0

    def setZero(self, step, rpm, cmd_timeout = 10, zero_timeout = 10000):
        print("Enable Motor")
        for i in range(cmd_timeout):
            ret = self.setEnablePin(True)
            if ret == 1:
                break
            if i == cmd_timeout-1:
                return -1
        done = False
        count = 0 
        while not done :
            self.setMotorRelativePose(rpm,step)
            #print(self.calDelayTime(rpm,step))
            time.sleep(self.calDelayTime(rpm,step))

            status, ret = self.readShaftStatus()
            if ret != 1:
                continue
            if status == 1: # blocked
                done = True

            count += 1
            if count == zero_timeout:
                return 0

        print("Disable Motor")
        for i in range(cmd_timeout):
            ret = self.setEnablePin(False)
            if ret == 1:
                break
            if i == cmd_timeout-1:
                return -2

        print("Enable Motor")
        for i in range(cmd_timeout):
            ret = self.setEnablePin(True)
            if ret == 1:
                break
            if i == cmd_timeout-1:
                return -3

        return 1

    def tighten_until(
        self,
        stop_check,
        rpm: int,
        step: int,
        max_abs_pulses: int = 50000,
        sleep_margin_s: float = 0.01,
        cmd_timeout: int = 10,
    ):
        """
        stop_check() が True になるまで、step 分だけ相対移動を繰り返す。

        Args:
            stop_check: callable -> bool（Trueで停止）
            rpm: 回転数
            step: 相対パルス（正の値で閉じる方向）
            max_abs_pulses: 安全上限（累積パルスの絶対値）
            sleep_margin_s: 各ステップ後の追加待機時間
            cmd_timeout: enable retry 回数

        Returns:
            1: stop_check() により停止
            0: 安全上限で停止
            負値: 通信エラー等
        """
        print(f"[tighten_until] rpm={rpm}, step={step}, max_abs_pulses={max_abs_pulses}")

        # Enable
        for i in range(cmd_timeout):
            ret = self.setEnablePin(True)
            print(f"[tighten_until] setEnablePin ret={ret}")
            if ret == 1:
                break
            if i == cmd_timeout - 1:
                return -10  # enable failed

        sent_sum = 0
        count = 0
        while True:
            if stop_check():
                self.stopMotor()
                print(f"[tighten_until] Stopped by stop_check at count={count}")
                return 1

            if abs(sent_sum) >= max_abs_pulses:
                self.stopMotor()
                print(f"[tighten_until] Reached max_abs_pulses at count={count}")
                return 0

            ret = self.setMotorRelativePose(rpm, step)
            print(f"[tighten_until] count={count}, setMotorRelativePose ret={ret}, sent_sum={sent_sum}")
            if ret != 1:
                self.stopMotor()
                return ret

            sent_sum += step
            count += 1

            dt = self.calDelayTime(rpm, step) + sleep_margin_s
            if dt > 0:
                time.sleep(dt)