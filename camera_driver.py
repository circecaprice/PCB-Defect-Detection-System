"""
ZWO ASI camera driver wrapper.

Requirements:
    pip install zwoasi numpy
    
You also need the ZWO ASI SDK (libASICamera2.so on Linux). Download from:
    https://www.zwoastro.com/software
Place libASICamera2.so somewhere standard like /usr/local/lib/ and run ldconfig,
or pass the path to ZWOCamera.init_sdk().

On Linux you may need a udev rule (provided in the SDK) to access the camera without sudo.
"""

import os
import numpy as np

try:
    import zwoasi as asi
except ImportError:
    asi = None


class ZWOCamera:
    _sdk_initialized = False

    @classmethod
    def init_sdk(cls, library_path=None):
        if asi is None:
            raise RuntimeError("zwoasi not installed. Run: pip install zwoasi")
        if cls._sdk_initialized:
            return

        candidates = []
        if library_path:
            candidates.append(library_path)
        env = os.environ.get("ZWO_ASI_LIB")
        if env:
            candidates.append(env)
        candidates += [
            "/usr/local/lib/libASICamera2.so",
            "/usr/lib/libASICamera2.so",
            "/usr/lib/x86_64-linux-gnu/libASICamera2.so",
        ]
        for path in candidates:
            if path and os.path.exists(path):
                asi.init(path)
                cls._sdk_initialized = True
                return
        raise FileNotFoundError(
            "Could not find libASICamera2.so. Install the ZWO ASI SDK "
            "or set ZWO_ASI_LIB environment variable."
        )

    def __init__(self):
        self.camera = None
        self.info = None
        self.controls = None
        self.current_bin = 1

    # ---------- connection ----------
    def list_cameras(self):
        self.init_sdk()
        return asi.list_cameras()

    def connect(self, index=0):
        self.init_sdk()
        self.camera = asi.Camera(index)
        self.info = self.camera.get_camera_property()
        self.controls = self.camera.get_controls()

        # sensible defaults
        if "BandWidth" in self.controls:
            self.camera.set_control_value(
                asi.ASI_BANDWIDTHOVERLOAD,
                self.controls["BandWidth"]["MinValue"],
            )
        self.camera.disable_dark_subtract()

        # Default ROI: full sensor, bin 1, RAW8 for fast live view
        self.set_roi(bin_val=1, image_type="RAW8")
        return self.info

    def disconnect(self):
        if self.camera:
            try: self.camera.stop_video_capture()
            except Exception: pass
            try: self.camera.stop_exposure()
            except Exception: pass
            try: self.camera.close()
            except Exception: pass
        self.camera = None

    def is_connected(self):
        return self.camera is not None

    # ---------- controls ----------
    def set_gain(self, gain):
        self.camera.set_control_value(asi.ASI_GAIN, int(gain))

    def get_gain(self):
        return self.camera.get_control_value(asi.ASI_GAIN)[0]

    def set_exposure_us(self, exposure_us):
        self.camera.set_control_value(asi.ASI_EXPOSURE, int(exposure_us))

    def get_exposure_us(self):
        return self.camera.get_control_value(asi.ASI_EXPOSURE)[0]

    def get_temperature_c(self):
        return self.camera.get_control_value(asi.ASI_TEMPERATURE)[0] / 10.0

    def set_cooler(self, on, target_c=None):
        if "CoolerOn" in self.controls:
            self.camera.set_control_value(asi.ASI_COOLER_ON, 1 if on else 0)
            if on and target_c is not None and "TargetTemp" in self.controls:
                self.camera.set_control_value(asi.ASI_TARGET_TEMP, int(target_c))

    def get_cooler_power(self):
        if "CoolerPowerPerc" in self.controls:
            return self.camera.get_control_value(asi.ASI_COOLER_POWER_PERC)[0]
        return None

    def has_cooler(self):
        return "CoolerOn" in (self.controls or {})

    # ---------- ROI / binning ----------
    def set_roi(self, bin_val=1, image_type="RAW8"):
        """Full sensor at the requested bin. image_type: RAW8 / RAW16 / RGB24 / Y8"""
        max_w = self.info["MaxWidth"]
        max_h = self.info["MaxHeight"]
        # binned dimensions, rounded to required multiples
        w = (max_w // bin_val) & ~7   # multiple of 8
        h = (max_h // bin_val) & ~1   # multiple of 2

        type_map = {
            "RAW8":  asi.ASI_IMG_RAW8,
            "RAW16": asi.ASI_IMG_RAW16,
            "RGB24": asi.ASI_IMG_RGB24,
            "Y8":    asi.ASI_IMG_Y8,
        }
        img_type = type_map.get(image_type.upper(), asi.ASI_IMG_RAW8)

        # stop any running capture first
        try: self.camera.stop_video_capture()
        except Exception: pass

        self.camera.set_roi(width=w, height=h, bins=bin_val, image_type=img_type)
        self.current_bin = bin_val
        return w, h

    # ---------- capture ----------
    def start_video(self):
        self.camera.start_video_capture()

    def stop_video(self):
        try:
            self.camera.stop_video_capture()
        except Exception:
            pass

    def capture_video_frame(self, timeout_ms=2000):
        return self.camera.capture_video_frame(timeout=timeout_ms)

    def capture_still(self):
        return self.camera.capture()

    # ---------- shutter (mechanical, if supported; soft shutter otherwise) ----------
    def trigger_shutter(self, exposure_us=None):
        """Single still exposure. ASI6200 has no mechanical shutter; this is just a timed exposure."""
        if exposure_us is not None:
            self.set_exposure_us(exposure_us)
        return self.capture_still()