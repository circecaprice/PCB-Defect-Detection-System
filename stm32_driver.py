"""
Simple USB CDC (virtual COM port) driver for the STM32F401RE.
The STM32 firmware should expose a line-based command protocol over USB CDC.
This driver is intentionally generic so it can be used both for raw USB-to-UART
loopback testing and for sending LED / GPIO commands.
"""

import serial
import threading
import time
from collections import deque


class STM32Driver:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

        self._rx_buffer = deque(maxlen=500)
        self._rx_thread = None
        self._stop_flag = False
        self._lock = threading.Lock()

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self._stop_flag = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def disconnect(self):
        self._stop_flag = True
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self.ser:
            try: self.ser.close()
            except Exception: pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def _rx_loop(self):
        buf = b""
        while not self._stop_flag and self.ser:
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode(errors="replace").strip()
                        with self._lock:
                            self._rx_buffer.append((time.time(), text))
            except Exception:
                break

    def send(self, data):
        if not self.is_connected():
            raise RuntimeError("STM32 not connected")
        if isinstance(data, str):
            if not data.endswith("\n"):
                data += "\n"
            data = data.encode()
        self.ser.write(data)

    def send_raw_bytes(self, b):
        if not self.is_connected():
            raise RuntimeError("STM32 not connected")
        self.ser.write(b)

    def pop_lines(self):
        with self._lock:
            lines = list(self._rx_buffer)
            self._rx_buffer.clear()
        return lines

    # ---------- LED helpers (firmware-side protocol assumption) ----------
    # Suggested protocol on the STM32: ASCII commands terminated with '\n'.
    #   LED <channel> <0..255>     -> set PWM duty
    #   LED ALL <0..255>           -> set all channels
    #   PING                       -> firmware replies "PONG"
    #   ECHO <text>                -> firmware echoes
    def set_led(self, channel, brightness):
        brightness = max(0, min(255, int(brightness)))
        self.send(f"LED {int(channel)} {brightness}")

    def set_all_leds(self, brightness):
        brightness = max(0, min(255, int(brightness)))
        self.send(f"LED ALL {brightness}")

    def ping(self):
        self.send("PING")