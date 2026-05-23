import sys
import time
import os
import glob
from datetime import datetime
import numpy as np

import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QPushButton, QTabWidget, QLineEdit, QCheckBox, QGroupBox,
    QGridLayout, QMessageBox, QSpinBox, QDoubleSpinBox, QTextEdit,
    QSizePolicy, QScrollArea, QSlider
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap, QIcon

from wemacro_driver import WeMacroDriver
from camera_driver import ZWOCamera
from stm32_driver import STM32Driver


# =============================================================================
# Live View thread
# =============================================================================
class LiveViewThread(QThread):
    frame_ready = pyqtSignal(np.ndarray, float)  # frame, fps
    error = pyqtSignal(str)

    def __init__(self, camera: ZWOCamera):
        super().__init__()
        self.camera = camera
        self._running = False

    def run(self):
        self._running = True
        try:
            self.camera.start_video()
        except Exception as e:
            self.error.emit(f"start_video failed: {e}")
            return

        last = time.time()
        frames = 0
        fps = 0.0
        while self._running:
            try:
                frame = self.camera.capture_video_frame(timeout_ms=2000)
            except Exception as e:
                self.error.emit(f"frame error: {e}")
                break
            frames += 1
            now = time.time()
            if now - last >= 0.5:
                fps = frames / (now - last)
                frames = 0
                last = now
            self.frame_ready.emit(frame, fps)

        self.camera.stop_video()

    def stop(self):
        self._running = False
        self.wait(2000)


# =============================================================================
# Rail tab (your existing UI, lightly refactored into a QWidget)
# =============================================================================
class RailTab(QWidget):
    def __init__(self):
        super().__init__()
        self.driver = None
        self.current_pos = 0.0
        self.start_pos = None
        self.end_pos = None
        self.pulses_per_mm = 6400
        self.backlash_val = 0.22

        layout = QVBoxLayout(self)
        self._build_connection(layout)
        self._build_tabs(layout)
        self._build_shutter(layout)

    def _build_connection(self, parent_layout):
        group = QGroupBox("Rail Connection")
        layout = QHBoxLayout(group)
        self.port_combo = QComboBox()
        self._refresh_ports()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        self.btn_save = QPushButton("Save")
        self.btn_save.clicked.connect(self._save_config)
        layout.addWidget(QLabel("Port:"))
        layout.addWidget(self.port_combo, 1)
        layout.addWidget(self.btn_refresh)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.btn_save)
        parent_layout.addWidget(group)

    def _refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(p.device)

    def _build_tabs(self, parent_layout):
        self.tabs = QTabWidget()

        # --- Distance ---
        dist = QWidget(); dl = QGridLayout(dist)
        self.dist_total = QLineEdit("10.0")
        self.dist_step = QLineEdit("0.1")
        self.dist_total_steps = QLabel("Total steps: 100")
        dl.addWidget(QLabel("Total distance (mm):"), 0, 0); dl.addWidget(self.dist_total, 0, 1)
        dl.addWidget(QLabel("Step length (mm):"), 1, 0);    dl.addWidget(self.dist_step, 1, 1)
        dl.addWidget(self.dist_total_steps, 2, 0, 1, 2)
        self.dist_total.textChanged.connect(self._update_dist_steps)
        self.dist_step.textChanged.connect(self._update_dist_steps)

        # --- Start/End ---
        ste = QWidget(); sl = QGridLayout(ste)
        self.ste_step = QLineEdit("0.1")
        self.ste_total_steps = QLabel("Total steps: 0")
        self.btn_as_start = QPushButton("As Start"); self.btn_as_start.clicked.connect(self._set_start)
        self.btn_as_end   = QPushButton("As End");   self.btn_as_end.clicked.connect(self._set_end)
        sl.addWidget(QLabel("Step length (mm):"), 0, 0); sl.addWidget(self.ste_step, 0, 1)
        sl.addWidget(self.btn_as_start, 1, 0); sl.addWidget(self.btn_as_end, 1, 1)
        sl.addWidget(self.ste_total_steps, 2, 0, 1, 2)
        self.ste_step.textChanged.connect(self._update_ste_steps)

        # --- Config ---
        cfg = QWidget(); cl = QGridLayout(cfg)
        self.cfg_pulses = QLineEdit("6400")
        self.cfg_backlash = QLineEdit("0.22")
        self.cfg_last_cmd = QLineEdit("None"); self.cfg_last_cmd.setReadOnly(True)
        cl.addWidget(QLabel("Motor Pulses/mm:"), 0, 0); cl.addWidget(self.cfg_pulses, 0, 1)
        cl.addWidget(QLabel("Backlash (mm):"), 1, 0);   cl.addWidget(self.cfg_backlash, 1, 1)
        cl.addWidget(QLabel("Last Cmd (HEX):"), 2, 0);  cl.addWidget(self.cfg_last_cmd, 2, 1)
        self.cfg_pulses.textChanged.connect(self._update_cfg_vars)
        self.cfg_backlash.textChanged.connect(self._update_cfg_vars)

        self.tabs.addTab(dist, "Distance")
        self.tabs.addTab(ste,  "Start to End")
        self.tabs.addTab(cfg,  "Config")
        parent_layout.addWidget(self.tabs)

        # Controls
        ctrl = QGroupBox("Controls"); g = QGridLayout(ctrl)
        self.btn_fwd       = QPushButton("Forward")
        self.btn_bwd       = QPushButton("Backward")
        self.btn_step_fwd  = QPushButton("Step Forward")
        self.btn_run       = QPushButton("Run")
        self.btn_stop      = QPushButton("Stop")
        self.btn_calibrate = QPushButton("Calibrate")
        self.chk_return    = QCheckBox("Return")
        self.chk_beep      = QCheckBox("Beep after done")
        self.btn_fwd.clicked.connect(lambda: self._move_manual('fwd'))
        self.btn_bwd.clicked.connect(lambda: self._move_manual('bwd'))
        self.btn_step_fwd.clicked.connect(lambda: self._move_manual('step'))
        self.btn_run.clicked.connect(self._run)
        self.btn_stop.clicked.connect(self._stop)
        self.btn_calibrate.clicked.connect(self._calibrate)
        g.addWidget(self.btn_fwd, 0, 0); g.addWidget(self.btn_bwd, 0, 1); g.addWidget(self.btn_step_fwd, 0, 2)
        g.addWidget(self.btn_run, 1, 0); g.addWidget(self.btn_stop, 1, 1); g.addWidget(self.btn_calibrate, 1, 2)
        g.addWidget(self.chk_return, 2, 0); g.addWidget(self.chk_beep, 2, 1)
        parent_layout.addWidget(ctrl)

    def _build_shutter(self, parent_layout):
        group = QGroupBox("Rail Shutter Parameters")
        layout = QGridLayout(group)
        self.shutter_wait = QLineEdit("0")
        self.shutter_per_step = QLineEdit("0")
        self.shutter_interval = QLineEdit("0")
        layout.addWidget(QLabel("Waiting time:"), 0, 0); layout.addWidget(self.shutter_wait, 0, 1)
        layout.addWidget(QLabel("Shutters/step:"), 1, 0); layout.addWidget(self.shutter_per_step, 1, 1)
        layout.addWidget(QLabel("Interval:"), 2, 0);      layout.addWidget(self.shutter_interval, 2, 1)
        self.btn_shutter = QPushButton("Trigger Rail Shutter")
        self.btn_shutter.clicked.connect(self._shutter)
        layout.addWidget(self.btn_shutter, 3, 0, 1, 2)
        parent_layout.addWidget(group)

    # ---- helpers ----
    def _check_connection(self):
        if not self.driver or not self.driver.is_connected():
            QMessageBox.warning(self, "Error", "Not connected to rail")
            return False
        return True

    def _log_cmd(self, b):
        if b: self.cfg_last_cmd.setText(" ".join(f"{x:02X}" for x in b))

    def _toggle_connection(self):
        if self.driver and self.driver.is_connected():
            self.driver.disconnect()
            self.btn_connect.setText("Connect")
            return
        port = self.port_combo.currentText()
        if not port:
            return
        try: pulses = int(self.cfg_pulses.text())
        except: pulses = 6400
        try:
            self.driver = WeMacroDriver(port, pulses_per_mm=pulses)
            self.driver.connect()
            self.btn_connect.setText("Disconnect")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to connect: {e}")

    def _update_cfg_vars(self):
        try:
            self.pulses_per_mm = int(self.cfg_pulses.text())
            if self.driver: self.driver.pulses_per_mm = self.pulses_per_mm
        except: pass
        try: self.backlash_val = float(self.cfg_backlash.text())
        except: pass

    def _update_dist_steps(self):
        try:
            total = float(self.dist_total.text()); step = float(self.dist_step.text())
            if step > 0: self.dist_total_steps.setText(f"Total steps: {int(total/step)}")
        except ValueError: pass

    def _update_ste_steps(self):
        if self.start_pos is None or self.end_pos is None: return
        try:
            step = float(self.ste_step.text())
            if step > 0:
                d = abs(self.end_pos - self.start_pos)
                self.ste_total_steps.setText(f"Total steps: {int(d/step)}")
        except ValueError: pass

    def _set_start(self):
        self.start_pos = self.current_pos
        QMessageBox.information(self, "Start", f"Start at {self.start_pos:.3f} mm")
        self._update_ste_steps()

    def _set_end(self):
        if self.start_pos is None:
            QMessageBox.warning(self, "Error", "Set the start point first."); return
        self.end_pos = self.current_pos
        d = self.end_pos - self.start_pos
        if not self._check_connection(): return
        try:
            if d > 0:   self.driver.back(d); self.current_pos -= d
            elif d < 0: self.driver.go(-d);  self.current_pos -= d
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Return failed: {e}")
        self._update_ste_steps()

    def _move_manual(self, direction):
        if not self._check_connection(): return
        try:
            is_dist = self.tabs.currentIndex() == 0
            step = float(self.dist_step.text() if is_dist else self.ste_step.text())
            wait = int(self.shutter_wait.text() or 0)
            per_step = int(self.shutter_per_step.text() or 0)
            interval = int(self.shutter_interval.text() or 0)
            if direction in ('fwd', 'step'):
                _, cmd = self.driver.go(step, wait, per_step, interval); self.current_pos += step
            else:
                _, cmd = self.driver.back(step, wait, per_step, interval); self.current_pos -= step
            self._log_cmd(cmd)
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def _run(self):
        if not self._check_connection(): return
        try:
            is_dist = self.tabs.currentIndex() == 0
            if is_dist:
                total = float(self.dist_total.text()); step = float(self.dist_step.text())
                steps = int(total/step)
            else:
                if self.start_pos is None or self.end_pos is None:
                    QMessageBox.warning(self, "Error", "Start/End not set"); return
                step = float(self.ste_step.text())
                steps = int(abs(self.end_pos - self.start_pos)/step)
            wait = int(self.shutter_wait.text() or 0)
            per_step = int(self.shutter_per_step.text() or 0)
            interval = int(self.shutter_interval.text() or 0)
            _, cmd = self.driver.run(steps,
                                     beep_after_done=self.chk_beep.isChecked(),
                                     back_after_done=self.chk_return.isChecked(),
                                     shutter_waiting_time=wait,
                                     shutters_per_step=per_step,
                                     shutter_interval=interval)
            self._log_cmd(cmd)
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def _stop(self):
        if not self._check_connection(): return
        try: _, cmd = self.driver.stop(); self._log_cmd(cmd)
        except Exception as e: QMessageBox.warning(self, "Error", str(e))

    def _calibrate(self):
        if not self._check_connection(): return
        try:
            _, cmd = self.driver.back(self.backlash_val); self._log_cmd(cmd)
            QTimer.singleShot(1000, self._calibrate_step2)
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def _calibrate_step2(self):
        if self.driver:
            _, cmd = self.driver.go(self.backlash_val); self._log_cmd(cmd)

    def _shutter(self):
        if not self._check_connection(): return
        try: _, cmd = self.driver.shutter(); self._log_cmd(cmd)
        except Exception as e: QMessageBox.warning(self, "Error", str(e))

    def _save_config(self):
        if not self._check_connection(): return
        try:
            is_dist = self.tabs.currentIndex() == 0
            step = float(self.dist_step.text() if is_dist else self.ste_step.text())
            _, cmd = self.driver.write_config(step,
                                              beep_after_done=self.chk_beep.isChecked(),
                                              back_after_done=self.chk_return.isChecked(),
                                              step_length_unit_mm=True)
            self._log_cmd(cmd)
            QMessageBox.information(self, "Saved", "Config saved")
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))


# =============================================================================
# Camera tab
# =============================================================================
class CameraTab(QWidget):
    def __init__(self):
        super().__init__()
        self.camera = ZWOCamera()
        self.live_thread = None
        self._last_frame = None

        self._ensure_capture_dir()
        self.recent_capture_paths = []

        root = QHBoxLayout(self)

        # Left: controls
        left = QVBoxLayout()
        root.addLayout(left, 1)

        # Right: live view
        right = QVBoxLayout()
        self.view = QLabel("No image")
        self.view.setAlignment(Qt.AlignCenter)
        self.view.setMinimumSize(480, 320)
        self.view.setMaximumSize(640, 480)
        self.view.setStyleSheet("background-color:#111; color:#888;")
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fps_label = QLabel("FPS: -")
        right.addWidget(self.view, 1)
        right.addWidget(self.fps_label)
        root.addLayout(right, 2)

        # ----- Connection -----
        conn = QGroupBox("Camera Connection")
        cl = QGridLayout(conn)
        self.cam_combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh"); self.btn_refresh.clicked.connect(self._refresh)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self._toggle_connect)
        self.info_label = QLabel("Not connected")
        cl.addWidget(QLabel("Camera:"), 0, 0); cl.addWidget(self.cam_combo, 0, 1, 1, 2)
        cl.addWidget(self.btn_refresh, 1, 1); cl.addWidget(self.btn_connect, 1, 2)
        cl.addWidget(self.info_label, 2, 0, 1, 3)
        left.addWidget(conn)

        # ----- Exposure / Gain -----
        eg = QGroupBox("Exposure & Gain")
        g = QGridLayout(eg)
        self.exposure_us = QSpinBox(); self.exposure_us.setRange(32, 600_000_000)
        self.exposure_us.setSuffix(" µs"); self.exposure_us.setValue(10_000)
        self.exposure_us.valueChanged.connect(self._apply_exposure)
        self.gain = QSpinBox(); self.gain.setRange(0, 500); self.gain.setValue(100)
        self.gain.valueChanged.connect(self._apply_gain)
        g.addWidget(QLabel("Exposure:"), 0, 0); g.addWidget(self.exposure_us, 0, 1)
        g.addWidget(QLabel("Gain:"),     1, 0); g.addWidget(self.gain, 1, 1)
        left.addWidget(eg)

        # ----- Binning / ROI -----
        bg = QGroupBox("Binning / Image Type")
        bl = QGridLayout(bg)
        self.bin_combo = QComboBox(); self.bin_combo.addItems(["1", "2", "4"])
        self.type_combo = QComboBox(); self.type_combo.addItems(["RAW8", "RAW16"])
        self.btn_apply_roi = QPushButton("Apply"); self.btn_apply_roi.clicked.connect(self._apply_roi)
        bl.addWidget(QLabel("Bin (NxN):"), 0, 0); bl.addWidget(self.bin_combo, 0, 1)
        bl.addWidget(QLabel("Image type:"), 1, 0); bl.addWidget(self.type_combo, 1, 1)
        bl.addWidget(self.btn_apply_roi, 2, 0, 1, 2)
        left.addWidget(bg)

        # ----- Temperature / Cooler -----
        tg = QGroupBox("Temperature & Cooler")
        tl = QGridLayout(tg)
        self.temp_label = QLabel("Sensor: --- °C")
        self.cooler_chk = QCheckBox("Cooler ON"); self.cooler_chk.toggled.connect(self._apply_cooler)
        self.target_temp = QDoubleSpinBox(); self.target_temp.setRange(-40, 40); self.target_temp.setValue(0)
        self.target_temp.setSuffix(" °C"); self.target_temp.valueChanged.connect(self._apply_cooler)
        self.cooler_pwr = QLabel("Power: -")
        tl.addWidget(self.temp_label, 0, 0, 1, 2)
        tl.addWidget(self.cooler_chk, 1, 0)
        tl.addWidget(self.target_temp, 1, 1)
        tl.addWidget(self.cooler_pwr, 2, 0, 1, 2)
        left.addWidget(tg)

        # ----- Live view & shutter -----
        lv = QGroupBox("Capture")
        ll = QGridLayout(lv)
        self.btn_live = QPushButton("Start Live View"); self.btn_live.clicked.connect(self._toggle_live)
        self.btn_snap = QPushButton("Trigger Shutter (Snap)"); self.btn_snap.clicked.connect(self._snap)
        self.capture_folder_label = QLabel()
        self.capture_folder_label.setWordWrap(True)
        self.capture_folder_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        ll.addWidget(self.btn_live, 0, 0); ll.addWidget(self.btn_snap, 0, 1)
        ll.addWidget(QLabel("Saved to:"), 1, 0); ll.addWidget(self.capture_folder_label, 1, 1)
        left.addWidget(lv)

        recent = QGroupBox("Recent Captures")
        recent_layout = QVBoxLayout(recent)
        self.recent_scroll = QScrollArea()
        self.recent_scroll.setWidgetResizable(True)
        self.recent_container = QWidget()
        self.recent_container_layout = QHBoxLayout(self.recent_container)
        self.recent_container_layout.setContentsMargins(0, 0, 0, 0)
        self.recent_container_layout.setSpacing(5)
        self.recent_scroll.setWidget(self.recent_container)
        recent_layout.addWidget(self.recent_scroll)
        left.addWidget(recent)

        left.addStretch(1)

        # temperature poll
        self.temp_timer = QTimer(self); self.temp_timer.timeout.connect(self._poll_temp)
        self.temp_timer.start(1000)

        self._refresh()
        self.capture_folder_label.setText(self.capture_dir)
        self._load_recent_captures()

    def _ensure_capture_dir(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.capture_dir = os.path.join(base_dir, "captures")
        os.makedirs(self.capture_dir, exist_ok=True)

    def _save_capture(self, frame: np.ndarray):
        if frame.dtype == np.uint16:
            disp = (frame >> 8).astype(np.uint8)
        else:
            disp = frame

        if disp.ndim == 2:
            h, w = disp.shape
            img = QImage(disp.data, w, h, w, QImage.Format_Grayscale8)
        elif disp.ndim == 3 and disp.shape[2] == 3:
            h, w, _ = disp.shape
            img = QImage(disp.data, w, h, 3 * w, QImage.Format_RGB888)
        else:
            raise ValueError("Unsupported frame format for saving")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self.capture_dir, f"capture_{timestamp}.png")
        img.save(path)
        return path

    def _load_recent_captures(self):
        for i in reversed(range(self.recent_container_layout.count())):
            widget = self.recent_container_layout.takeAt(i).widget()
            if widget:
                widget.deleteLater()

        paths = sorted(glob.glob(os.path.join(self.capture_dir, "*.png")), key=os.path.getmtime, reverse=True)
        self.recent_capture_paths = paths[:8]
        for path in self.recent_capture_paths:
            button = QPushButton()
            button.setFixedSize(160, 120)
            button.setIcon(QIcon(path))
            button.setIconSize(button.size())
            button.setToolTip(os.path.basename(path))
            button.clicked.connect(lambda checked, p=path: self._load_recent_image(p))
            self.recent_container_layout.addWidget(button)

        self.recent_container_layout.addStretch(1)

    def _load_recent_image(self, path):
        pix = QPixmap(path)
        if not pix.isNull():
            self.view.setPixmap(pix.scaled(self.view.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            self.fps_label.setText(f"Loaded: {os.path.basename(path)}")

    # ---- camera lifecycle ----
    def _refresh(self):
        self.cam_combo.clear()
        try:
            for i, name in enumerate(self.camera.list_cameras()):
                self.cam_combo.addItem(f"{i}: {name}")
        except Exception as e:
            self.info_label.setText(f"SDK error: {e}")

    def _toggle_connect(self):
        if self.camera.is_connected():
            self._stop_live()
            self.camera.disconnect()
            self.btn_connect.setText("Connect")
            self.info_label.setText("Not connected")
            return
        idx = self.cam_combo.currentIndex()
        if idx < 0:
            QMessageBox.warning(self, "Error", "No camera selected"); return
        try:
            info = self.camera.connect(idx)
            self.info_label.setText(
                f"{info['Name']}  |  {info['MaxWidth']}x{info['MaxHeight']}  |  "
                f"PixelSize: {info.get('PixelSize', '?')} µm"
            )
            self.btn_connect.setText("Disconnect")
            # apply current control values
            self._apply_gain(); self._apply_exposure()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Connect failed: {e}")

    # ---- controls ----
    def _apply_gain(self):
        if self.camera.is_connected():
            try: self.camera.set_gain(self.gain.value())
            except Exception as e: print("gain:", e)

    def _apply_exposure(self):
        if self.camera.is_connected():
            try: self.camera.set_exposure_us(self.exposure_us.value())
            except Exception as e: print("exposure:", e)

    def _apply_roi(self):
        if not self.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera not connected"); return
        was_live = self.live_thread is not None
        if was_live: self._stop_live()
        try:
            bin_val = int(self.bin_combo.currentText())
            img_type = self.type_combo.currentText()
            w, h = self.camera.set_roi(bin_val=bin_val, image_type=img_type)
            self.info_label.setText(self.info_label.text().split("|")[0] +
                                    f"| ROI: {w}x{h} bin{bin_val} {img_type}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"ROI failed: {e}")
        if was_live: self._start_live()

    def _apply_cooler(self):
        if not self.camera.is_connected(): return
        if not self.camera.has_cooler(): return
        try:
            self.camera.set_cooler(self.cooler_chk.isChecked(),
                                   target_c=self.target_temp.value())
        except Exception as e:
            print("cooler:", e)

    def _poll_temp(self):
        if not self.camera.is_connected(): return
        try:
            t = self.camera.get_temperature_c()
            self.temp_label.setText(f"Sensor: {t:.1f} °C")
            pwr = self.camera.get_cooler_power()
            if pwr is not None:
                self.cooler_pwr.setText(f"Cooler power: {pwr}%")
        except Exception:
            pass

    # ---- live view ----
    def _toggle_live(self):
        if self.live_thread is None: self._start_live()
        else: self._stop_live()

    def _start_live(self):
        if not self.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera not connected"); return
        self.live_thread = LiveViewThread(self.camera)
        self.live_thread.frame_ready.connect(self._on_frame)
        self.live_thread.error.connect(lambda m: print("live err:", m))
        self.live_thread.start()
        self.btn_live.setText("Stop Live View")

    def _stop_live(self):
        if self.live_thread:
            self.live_thread.stop()
            self.live_thread = None
        self.btn_live.setText("Start Live View")

    def _on_frame(self, frame: np.ndarray, fps: float):
        self._last_frame = frame
        self.fps_label.setText(f"FPS: {fps:.1f}   shape: {frame.shape}  dtype: {frame.dtype}")
        self._show_frame(frame)

    def _show_frame(self, frame: np.ndarray):
        # Normalize to 8-bit for display
        if frame.dtype == np.uint16:
            disp = (frame >> 8).astype(np.uint8)
        else:
            disp = frame
        if disp.ndim == 2:
            h, w = disp.shape
            img = QImage(disp.data, w, h, w, QImage.Format_Grayscale8)
        elif disp.ndim == 3 and disp.shape[2] == 3:
            h, w, _ = disp.shape
            img = QImage(disp.data, w, h, 3 * w, QImage.Format_RGB888)
        else:
            return
        # scale to label size
        pix = QPixmap.fromImage(img).scaled(self.view.size(),
                                            Qt.KeepAspectRatio,
                                            Qt.FastTransformation)
        self.view.setPixmap(pix)

    def _snap(self):
        if not self.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera not connected"); return
        was_live = self.live_thread is not None
        if was_live: self._stop_live()
        try:
            frame = self.camera.trigger_shutter()
            self._show_frame(frame)
            saved_path = self._save_capture(frame)
            self.capture_folder_label.setText(self.capture_dir)
            self._load_recent_captures()
            QMessageBox.information(self, "Snap",
                                    f"Captured {frame.shape} ({frame.dtype})\nSaved to:\n{saved_path}")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Snap failed: {e}")
        if was_live: self._start_live()

    def closeEvent(self, ev):
        self._stop_live()
        self.camera.disconnect()
        super().closeEvent(ev)


# =============================================================================
# STM32 tab — USB <-> UART loopback + LED controls
# =============================================================================
class STM32Tab(QWidget):
    """
    Sanity-test path for the STM32F401RE acting as a USB CDC <-> UART bridge.
    Use this tab to:
      - Connect to /dev/ttyACMx exposed by the STM32 USB CDC interface
      - Send arbitrary text or hex bytes and view replies
      - Drive LED channels via a small ASCII protocol (see stm32_driver.py)
    """
    def __init__(self):
        super().__init__()
        self.driver = None
        layout = QVBoxLayout(self)

        # Connection
        conn = QGroupBox("STM32 USB CDC Connection")
        cl = QHBoxLayout(conn)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        for b in ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("115200")
        self.btn_refresh = QPushButton("Refresh"); self.btn_refresh.clicked.connect(self._refresh)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self._toggle_connect)
        cl.addWidget(QLabel("Port:")); cl.addWidget(self.port_combo, 1)
        cl.addWidget(QLabel("Baud:")); cl.addWidget(self.baud_combo)
        cl.addWidget(self.btn_refresh); cl.addWidget(self.btn_connect)
        layout.addWidget(conn)

        # Send box
        send = QGroupBox("Send (text or hex). Hex example:  HEX 01 02 0A FF")
        sl = QHBoxLayout(send)
        self.send_edit = QLineEdit()
        self.send_edit.returnPressed.connect(self._send_clicked)
        self.btn_send = QPushButton("Send"); self.btn_send.clicked.connect(self._send_clicked)
        self.btn_ping = QPushButton("PING");  self.btn_ping.clicked.connect(self._ping)
        sl.addWidget(self.send_edit, 1); sl.addWidget(self.btn_send); sl.addWidget(self.btn_ping)
        layout.addWidget(send)

        # LED controls (assumes firmware implements "LED <ch> <0..255>" / "LED ALL <0..255>")
        led = QGroupBox("LED Control (PWM 0..255 per channel)")
        gl = QGridLayout(led)
        self.led_sliders = []
        for ch in range(4):
            s = QSlider(Qt.Horizontal); s.setRange(0, 255); s.setValue(0)
            lbl = QLabel("0")
            s.valueChanged.connect(lambda v, l=lbl, c=ch: (l.setText(str(v)), self._set_led(c, v)))
            gl.addWidget(QLabel(f"CH {ch}:"), ch, 0)
            gl.addWidget(s, ch, 1); gl.addWidget(lbl, ch, 2)
            self.led_sliders.append(s)
        self.all_slider = QSlider(Qt.Horizontal); self.all_slider.setRange(0, 255)
        self.all_label = QLabel("0")
        self.all_slider.valueChanged.connect(self._set_all_leds)
        gl.addWidget(QLabel("ALL:"), 4, 0); gl.addWidget(self.all_slider, 4, 1); gl.addWidget(self.all_label, 4, 2)
        layout.addWidget(led)

        # Console
        con = QGroupBox("Console (RX from STM32)")
        v = QVBoxLayout(con)
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setStyleSheet("background:#101010;color:#cfc;font-family:monospace;")
        btns = QHBoxLayout()
        self.btn_clear = QPushButton("Clear"); self.btn_clear.clicked.connect(self.console.clear)
        btns.addStretch(1); btns.addWidget(self.btn_clear)
        v.addWidget(self.console); v.addLayout(btns)
        layout.addWidget(con, 1)

        # poll RX
        self.rx_timer = QTimer(self); self.rx_timer.timeout.connect(self._drain_rx)
        self.rx_timer.start(100)

        self._refresh()

    def _refresh(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            desc = p.description or ""
            self.port_combo.addItem(f"{p.device}  ({desc})", p.device)

    def _toggle_connect(self):
        if self.driver and self.driver.is_connected():
            self.driver.disconnect(); self.driver = None
            self.btn_connect.setText("Connect")
            self._log("-- disconnected --")
            return
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "Error", "No ports available"); return
        port = self.port_combo.currentData() or self.port_combo.currentText().split()[0]
        baud = int(self.baud_combo.currentText())
        try:
            self.driver = STM32Driver(port, baudrate=baud)
            self.driver.connect()
            self.btn_connect.setText("Disconnect")
            self._log(f"-- connected {port} @ {baud} --")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Connect failed: {e}")

    def _send_clicked(self):
        if not self._check(): return
        text = self.send_edit.text()
        if not text: return
        try:
            if text.strip().upper().startswith("HEX "):
                hex_part = text.split(None, 1)[1].replace(",", " ")
                raw = bytes(int(b, 16) for b in hex_part.split())
                self.driver.send_raw_bytes(raw)
                self._log(f"TX (hex): {' '.join(f'{b:02X}' for b in raw)}")
            else:
                self.driver.send(text)
                self._log(f"TX: {text}")
            self.send_edit.clear()
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def _ping(self):
        if not self._check(): return
        try:
            self.driver.ping(); self._log("TX: PING")
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))

    def _set_led(self, ch, value):
        if not self.driver or not self.driver.is_connected(): return
        try:
            self.driver.set_led(ch, value)
        except Exception as e:
            self._log(f"!! LED err: {e}")

    def _set_all_leds(self, value):
        self.all_label.setText(str(value))
        for s in self.led_sliders:
            s.blockSignals(True); s.setValue(value); s.blockSignals(False)
        if not self.driver or not self.driver.is_connected(): return
        try:
            self.driver.set_all_leds(value)
        except Exception as e:
            self._log(f"!! LED err: {e}")

    def _drain_rx(self):
        if not self.driver or not self.driver.is_connected(): return
        for ts, line in self.driver.pop_lines():
            self._log(f"RX: {line}")

    def _log(self, text):
        self.console.append(text)

    def _check(self):
        if not self.driver or not self.driver.is_connected():
            QMessageBox.warning(self, "Error", "STM32 not connected"); return False
        return True

    def closeEvent(self, ev):
        if self.driver: self.driver.disconnect()
        super().closeEvent(ev)


# =============================================================================
# Main window
# =============================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Camera Rig Control")
        self.setMinimumSize(1100, 720)

        self.rail_tab   = RailTab()
        self.camera_tab = CameraTab()
        self.stm32_tab  = STM32Tab()

        self.rail_tab.setMinimumHeight(200)
        self.camera_tab.setMinimumHeight(360)
        self.stm32_tab.setMinimumHeight(200)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        layout.addWidget(self.rail_tab)
        layout.addWidget(self.camera_tab, 3)
        layout.addWidget(self.stm32_tab)
        layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        self.setCentralWidget(scroll)

    def closeEvent(self, ev):
        self.camera_tab.closeEvent(ev)
        self.stm32_tab.closeEvent(ev)
        super().closeEvent(ev)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())