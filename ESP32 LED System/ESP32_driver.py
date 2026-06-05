"""
WLED-over-USB serial driver.

The ESP32 runs WLED firmware, which accepts its JSON API over the USB serial
port (115200 baud by default): send a JSON object terminated by a newline, e.g.
    {"on":true,"bri":128}\\n
and send
    {"v":true}\\n
to receive the full {"state":..,"info":..} object back (newline-terminated).

NOTE on naming: the file is ESP32_driver.py (kept from the project scaffold) but
the class is WLEDDriver because the protocol it speaks is WLED's JSON API, not a
bespoke ESP32 protocol.

Caveat: if WLED's LED output is assigned to GPIO1/GPIO3 (the UART0 TX/RX pins),
the USB serial console is unavailable and no replies will arrive.

Mirrors the structure of the sibling stm32_driver.py: a background daemon thread
reads bytes, splits on '\\n', and pushes (timestamp, line) tuples into a bounded
deque that the UI drains via pop_lines(). On top of that, query()/get_state()
provide a synchronous request/response read for state without racing the reader.
"""

import json
import time
import threading
from collections import deque

import serial


class WLEDDriver:
    def __init__(self, port, baudrate=115200, timeout=0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

        self._rx_buffer = deque(maxlen=500)   # (time.time(), text) for the console
        self._rx_thread = None
        self._stop_flag = False
        self._lock = threading.Lock()

        # Request/response gate: while _query_mode is True the RX thread also
        # copies each line into _query_lines so query() can consume the reply
        # without becoming a second reader of the serial port.
        self._query_mode = False
        self._query_lines = deque()

    # ---------- lifecycle ----------
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
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    # ---------- RX ----------
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
                        if not text:
                            continue
                        with self._lock:
                            if self._query_mode:
                                self._query_lines.append(text)
                            self._rx_buffer.append((time.time(), text))
            except Exception:
                break

    def pop_lines(self):
        """Drain and return buffered console lines as a list of (ts, text)."""
        with self._lock:
            lines = list(self._rx_buffer)
            self._rx_buffer.clear()
        return lines

    # ---------- low-level TX ----------
    def send(self, data):
        if not self.is_connected():
            raise RuntimeError("WLED not connected")
        if isinstance(data, str):
            if not data.endswith("\n"):
                data += "\n"
            data = data.encode()
        self.ser.write(data)

    def send_json(self, obj):
        """Serialize a dict to compact JSON and send it (newline-terminated)."""
        self.send(json.dumps(obj, separators=(",", ":")))

    # ---------- request / response ----------
    def query(self, obj=None, timeout=1.0):
        """Send a command (default {"v":true}) and return the first parsed JSON
        dict containing a 'state' or 'info' key, or None on timeout.

        Briefly blocks the calling thread (polls in 10ms steps up to `timeout`).
        Intended for one-shot reads (connect-time sync / manual query), not a
        high-rate poll.
        """
        if not self.is_connected():
            raise RuntimeError("WLED not connected")
        if obj is None:
            obj = {"v": True}
        with self._lock:
            self._query_lines.clear()
            self._query_mode = True
        try:
            self.send_json(obj)
            deadline = time.time() + timeout
            while time.time() < deadline:
                with self._lock:
                    pending = list(self._query_lines)
                    self._query_lines.clear()
                for text in pending:
                    try:
                        d = json.loads(text)
                    except ValueError:
                        continue
                    if isinstance(d, dict) and ("state" in d or "info" in d):
                        return d
                time.sleep(0.01)
            return None
        finally:
            with self._lock:
                self._query_mode = False

    def get_state(self, timeout=1.0):
        """Return (state_dict, info_dict); either may be None on timeout."""
        d = self.query({"v": True}, timeout=timeout)
        if not d:
            return None, None
        return d.get("state"), d.get("info")

    # ---------- high-level commands ----------
    def set_power(self, on):
        # on may be True / False / "t" (toggle)
        self.send_json({"on": on})

    def set_brightness(self, bri):
        # Also assert on=true: WLED ignores brightness while the strip is off,
        # which otherwise makes the slider look like it does nothing.
        bri = _clamp(bri)
        msg = {"bri": bri}
        if bri > 0:
            msg["on"] = True
        self.send_json(msg)

    # The "normal" controls below also clear the freeze flag, so touching any of
    # them resumes live animation after a show_only() (which freezes the segment).
    def set_color(self, r, g, b, seg_id=0):
        self.send_json({"seg": [{"id": seg_id, "frz": False, "col": [[int(r), int(g), int(b)]]}]})

    def set_effect(self, fx, seg_id=0):
        self.send_json({"seg": [{"id": seg_id, "frz": False, "fx": int(fx)}]})

    def set_palette(self, pal, seg_id=0):
        self.send_json({"seg": [{"id": seg_id, "frz": False, "pal": int(pal)}]})

    def set_speed(self, sx, seg_id=0):
        self.send_json({"seg": [{"id": seg_id, "frz": False, "sx": _clamp(sx)}]})

    def set_intensity(self, ix, seg_id=0):
        self.send_json({"seg": [{"id": seg_id, "frz": False, "ix": _clamp(ix)}]})

    def recall_preset(self, ps):
        self.send_json({"ps": int(ps)})

    def save_preset(self, ps):
        self.send_json({"psave": int(ps)})

    def set_range(self, start, stop, seg_id=0):
        # stop is exclusive (WLED convention). NOTE: this only resizes the
        # segment; it does NOT turn off LEDs outside the range. For "light only
        # these LEDs, rest off" use clear() + show_only() instead.
        self.send_json({"seg": [{"id": seg_id, "start": int(start), "stop": int(stop)}]})

    def set_individual(self, pairs, seg_id=0):
        """pairs: iterable of (index, (r, g, b)). Emits WLED's segment 'i' API."""
        flat = []
        for idx, (r, g, b) in pairs:
            flat += [int(idx), [int(r), int(g), int(b)]]
        self.send_json({"seg": [{"id": seg_id, "i": flat}]})

    # ---------- "only these LEDs on" helpers ----------
    def clear(self, count, seg_id=0):
        """Black out the whole strip. Uses WLED's individual-LED ("i") range
        fill [start, stop, color], which also FREEZES the segment so the running
        effect won't repaint over it. Call before show_only()."""
        self.send_json({"on": True, "seg": [{"id": seg_id, "i": [0, int(count), [0, 0, 0]]}]})

    def show_only(self, pairs, seg_id=0):
        """Light ONLY the given (index, (r, g, b)) LEDs via the "i" API (which
        keeps the segment frozen so the pattern persists). Call clear() first so
        every other LED stays off."""
        flat = []
        for idx, (r, g, b) in pairs:
            flat += [int(idx), [int(r), int(g), int(b)]]
        self.send_json({"seg": [{"id": seg_id, "frz": True, "i": flat}]})

    def reset_all(self, count, seg_id=0):
        """Unfreeze, restore the full-strip segment, and show solid white so the
        whole strip is unmistakably ON again. (Pick an effect/color afterwards.)"""
        self.send_json({"on": True, "seg": [{"id": seg_id, "start": 0, "stop": int(count),
                                             "frz": False, "fx": 0, "col": [[255, 255, 255]]}]})


def _clamp(v, lo=0, hi=255):
    return max(lo, min(hi, int(v)))
