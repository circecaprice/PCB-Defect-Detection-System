import serial
import math
import time

def crc16(data):
    r = 0xFFFF
    for i in range(len(data)):
        r = r ^ data[i]
        for j in range(8):
            if (r & 0x01) == 1:
                r = (r >> 1) ^ 0xA001
            else:
                r = r >> 1
    return r

class WeMacroDriver:
    def __init__(self, port, baudrate=9600, pulses_per_mm=6400):
        self.port = port
        self.baudrate = baudrate
        self.pulses_per_mm = pulses_per_mm
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
        
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def _write_cmd(self, buf):
        if not self.is_connected():
            raise Exception("Serial port not open")
        crc = crc16(buf[:10])
        buf[10] = crc & 0xFF
        buf[11] = (crc >> 8) & 0xFF
        try:
            res = self.ser.write(bytes(buf))
            return res, bytes(buf)
        except Exception as e:
            raise Exception(f"Error writing to serial: {e}")

    def _get_pulses(self, length_mm):
        return int(math.ceil(length_mm * self.pulses_per_mm))

    def write_config(self, step_length_mm, beep_after_done=False, back_after_done=False, step_length_unit_mm=True):
        pulses = self._get_pulses(step_length_mm)
        b2 = 0x80
        if beep_after_done:
            b2 |= 0x02
        if back_after_done:
            b2 |= 0x08
            
        b3 = 0xFF if step_length_unit_mm else 0x00
        
        buf = [0xA5, 0x5A, b2, b3, 0x00, 0x00,
               (pulses >> 24) & 0xFF,
               (pulses >> 16) & 0xFF,
               (pulses >> 8) & 0xFF,
               pulses & 0xFF,
               0x00, 0x00]
        return self._write_cmd(buf)

    def go(self, length_mm, shutter_waiting_time=0, shutters_per_step=0, shutter_interval=0):
        pulses = self._get_pulses(length_mm)
        buf = [0xA5, 0x5A, 0x40,
               int(shutter_waiting_time), int(shutters_per_step), int(shutter_interval),
               (pulses >> 24) & 0xFF,
               (pulses >> 16) & 0xFF,
               (pulses >> 8) & 0xFF,
               pulses & 0xFF,
               0x00, 0x00]
        return self._write_cmd(buf)

    def back(self, length_mm, shutter_waiting_time=0, shutters_per_step=0, shutter_interval=0):
        pulses = self._get_pulses(length_mm)
        buf = [0xA5, 0x5A, 0x41,
               int(shutter_waiting_time), int(shutters_per_step), int(shutter_interval),
               (pulses >> 24) & 0xFF,
               (pulses >> 16) & 0xFF,
               (pulses >> 8) & 0xFF,
               pulses & 0xFF,
               0x00, 0x00]
        return self._write_cmd(buf)

    def wait_until_done(self, direction="forward"):
        expected = {
            "forward": bytes([0x5A, 0xA5, 0xF5]),
            "backward": bytes([0x5A, 0xA5, 0xF6]),
        }[direction]

        while self.ser.read(3) != expected:
            pass

    # position is not changed when run function occurs
    def run(self, steps, beep_after_done=False, back_after_done=False, shutter_waiting_time=0, shutters_per_step=0, shutter_interval=0):
        b2 = 0x10
        if beep_after_done:
            b2 |= 0x02
        if back_after_done:
            b2 |= 0x08
            
        buf = [0xA5, 0x5A, b2,
               int(shutter_waiting_time), int(shutters_per_step), int(shutter_interval),
               (steps >> 24) & 0xFF,
               (steps >> 16) & 0xFF,
               (steps >> 8) & 0xFF,
               steps & 0xFF,
               0x00, 0x00]
        return self._write_cmd(buf)

    def stop(self):
        buf = [0xA5, 0x5A, 0x20, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        return self._write_cmd(buf)

    def shutter(self):
        buf = [0xA5, 0x5A, 0x04, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        return self._write_cmd(buf)

    def get_duration(self, shutter_waiting_time, total_steps, shutters_per_step, shutter_interval, step_length, step_length_unit=0):
        a = float(shutter_waiting_time)
        b = float(total_steps)
        c = float(shutters_per_step)
        d = float(shutter_interval)
        e = float(step_length)
        if step_length_unit == 1:
            e_val = 0 if e <= 14 else math.ceil(e / 211)
            return b * (a + c * d + e_val) + a + c * d
        else:
            return b * (a + c * d + math.ceil(0.282 * e)) + a + c * d
