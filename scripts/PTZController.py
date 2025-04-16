import serial
import time

class PTZController:
    def __init__(self, port="", baudrate = 9600):
        try:
            self.ser = serial.Serial(port, baudrate,timeout=1)
            self.address=b'\x81'
            print(f"Connected to PTZ camera on {port}")
        except Exception as e:
            print(f"Failed to connect :{e}")
            self.ser = None
    
    def send_visca(self, command:bytes):
        full_cmd = self.address + command + b'\xFF'
        self.ser.write(full_cmd)
        time.sleep(0.1)

    def zoom_in(self):
        self.send_visca(b'\x01\x04\x07\x02')
    
    def zoom_out(self):
        self.send_visca(b'\x01\x04\x07\x03')

    def zoom_stop(self):
        self.send_visca(b'\x01\x04\x07\x00')

    def pan_tilt(self,pan_speed=0x18, tilt_speed=0x14, pan_dir=0x03, tilt_dir=0x03):
        cmd = b'\x01\x06\x01'+bytes([pan_speed, tilt_speed, pan_dir, tilt_dir])
        self.send_visca(cmd)
    
    def pan_rel_right(self, speed=0x10):
        self.pan_tilt(pan_speed=speed, pan_dir=0x02)

    def pan_rel_left(self,speed=0x10):
        self.pan_tilt(pan_speed=speed, pan_dir= 0x01)

    def tilt_up(self, speed=0x10):
        self.pan_tilt(tilt_speed=speed, tilt_dir=0x01)
    
    def tilt_down(self,speed=0x10):
        self.pan_tilt(tilt_speed=speed, tilt_dir=0x02)

    def stop(self):
        self.pan_tilt()

    def autofocus_on(self):
        self.send_visca(b'\x01\x04\x38\x02')

    def autofocus_off(self):
        self.send_visca(b'\x01\x04\x32\x03')

    def close(self):
        self.ser.close()
    
    def get_pan_angle(self):
        try:
            self.send_command(b'\x81\x09\x06\x12\xFF')  # VISCA 查询命令
            response = self.ser.read(9)  # 获取返回数据（9 字节）

            if len(response) >= 9:
                # 读取前四个字节（Pan 部分）
                pan_raw = response[0:4]
                pan_hex = ''.join([format(b, 'X') for b in pan_raw])
                pan_value = int(pan_hex, 16)

                # 将值映射为角度（-10000 ~ +10000 -> -170° ~ +170°）
                # 依据具体设备手册而定，以下为假设范围映射
                pan_angle = (pan_value - 0x8000) * (170.0 / 0x8000)
                return pan_angle
        except Exception as e:
            print(f"[PTZ] 获取 pan 失败: {e}")
        return 0.0
