import sys
import time
import os
import glob
from datetime import datetime
from scipy.optimize import curve_fit
import cv2

import numpy as np

import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QPushButton, QTabWidget, QLineEdit, QCheckBox, QGroupBox,
    QGridLayout, QMessageBox, QSpinBox, QDoubleSpinBox, QTextEdit,
    QSizePolicy, QScrollArea, QSlider, QProgressBar
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QRect
from PyQt5.QtGui import QImage, QPixmap, QIcon, QPainter, QPen

from wemacro_driver import WeMacroDriver
from camera_driver import ZWOCamera
from stm32_driver import STM32Driver

RAIL_MAX = 110
RAIL_MIN = 0

# =============================================================================
# Serial port enumeration
# =============================================================================
def list_serial_ports(show_all=False):
    """[(device, label)] for serial ports.

    By default only USB devices (ports reporting a USB VID) are returned, which
    hides legacy/virtual non-USB serial (e.g. /dev/ttyS*, built-in COM ports).
    Pass show_all=True to return every port.
    """
    out = []
    for p in serial.tools.list_ports.comports():
        if not show_all and p.vid is None:   # vid is None => not a USB device
            continue
        desc = (p.description or p.product or "").strip()
        label = f"{p.device}  ({desc})" if desc and desc.lower() != "n/a" else p.device
        out.append((p.device, label))
    return out


# =============================================================================
# Live View thread
# =============================================================================
class LiveViewThread(QThread):
    frame_ready = pyqtSignal(np.ndarray, float)  # frame, fps
    error = pyqtSignal(str)       # fatal: stream has stopped
    warning = pyqtSignal(str)     # non-fatal: stream stalled / recovering

    # Poll the SDK in short slices so we can react to stop() quickly and never
    # block the GUI thread for long. A frame that isn't ready yet just means
    # "keep polling", not "dropped".
    POLL_MS = 250
    # Conservative USB2 throughput (bytes/ms) with WLED-style low bandwidth: used
    # to estimate how long a full frame legitimately takes to transfer.
    TRANSFER_BYTES_PER_MS = 12_000
    # A stall = no complete frame within a whole budget window. Bounce the stream
    # after this many consecutive stalls; give up after the larger one.
    BOUNCE_AFTER = 2
    FATAL_AFTER = 6

    def __init__(self, camera: ZWOCamera):
        super().__init__()
        self.camera = camera
        self._running = False

    def _frame_budget_ms(self):
        # How long a single frame may legitimately take: exposure + sensor
        # readout/USB transfer, with generous headroom. We can afford a large
        # budget because we poll in POLL_MS slices, so stop() stays responsive.
        exp_ms = getattr(self.camera, "last_exposure_us", 10_000) / 1000.0
        nbytes = getattr(self.camera, "last_frame_bytes", 0) or 0
        transfer_ms = nbytes / self.TRANSFER_BYTES_PER_MS
        return max(3000.0, exp_ms * 2 + 500 + transfer_ms * 2.5)

    def _poll_frame(self):
        """Wait up to one frame budget for a complete frame, polling in short
        slices. Returns the frame, or None if the budget elapsed / we were
        asked to stop. A POLL_MS timeout just means 'not ready yet'."""
        budget = self._frame_budget_ms()
        waited = 0.0
        while self._running and waited < budget:
            try:
                return self.camera.capture_video_frame(timeout_ms=self.POLL_MS)
            except Exception:
                waited += self.POLL_MS
        return None

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
        stalls = 0

        try:
            while self._running:
                frame = self._poll_frame()
                if not self._running:
                    break
                if frame is None:
                    # No complete frame within a whole budget window — a real stall.
                    stalls += 1
                    if stalls >= self.FATAL_AFTER:
                        self.error.emit(f"live view stopped: no frames after {stalls} stalls")
                        break
                    self.warning.emit(f"waiting for frame… (stall {stalls})")
                    if stalls == self.BOUNCE_AFTER:
                        try:
                            self.camera.stop_video()
                            if not self._running:
                                break
                            self.camera.start_video()
                        except Exception as re:
                            self.error.emit(f"live view recovery failed: {re}")
                            break
                    continue

                stalls = 0
                frames += 1
                now = time.time()
                if now - last >= 0.5:
                    fps = frames / (now - last)
                    frames = 0
                    last = now
                self.frame_ready.emit(frame, fps)
        finally:
            try:
                self.camera.stop_video()
            except Exception:
                pass

    def stop(self):
        # The poll loop checks _running every POLL_MS and owns stop_video() in
        # its finally block, so the thread exits promptly without us touching the
        # SDK from this (GUI) thread — keeping the GUI responsive.
        self._running = False
        if not self.wait(3000):
            # Last resort: force the thread down rather than leak a running QThread.
            self.terminate()
            self.wait(1000)


# =============================================================================
# Snapshot thread (single still exposure, possibly long)
# =============================================================================
class SnapThread(QThread):
    """Runs one still exposure off the GUI thread so the UI stays responsive."""
    captured = pyqtSignal(np.ndarray)
    error = pyqtSignal(str)

    def __init__(self, camera: ZWOCamera):
        super().__init__()
        self.camera = camera

    def run(self):
        try:
            frame = self.camera.trigger_shutter()
        except Exception as e:
            self.error.emit(str(e))
            return
        self.captured.emit(frame)


# =============================================================================
# Auto-focus thread (rail sweep + focus scoring, off the GUI thread)
# =============================================================================
class AutoFocusThread(QThread):
    """Runs autofocus on a worker thread with simpler, clearer logic."""
    progress = pyqtSignal(str)  # status messages
    moved = pyqtSignal(float)  # rail position updates
    finished_focus = pyqtSignal(float)  # final focus position
    cancelled = pyqtSignal()
    error = pyqtSignal(str)

    def __init__(self, camera, rail_driver, camera_tab, start_pos, coarse_steps=None,
                 fine_window=10, fine_step=0.5, backlash=0.22, settle_ms=300,
                 flush_frames=2, samples=3):
        super().__init__()
        self.camera = camera
        self.rail = rail_driver
        self.camera_tab = camera_tab
        self.current_pos = float(start_pos)
        self.coarse_steps = coarse_steps if coarse_steps is not None else np.arange(RAIL_MIN, RAIL_MAX + 1, 10)
        self.fine_window = fine_window
        self.fine_step = float(fine_step)  # mm between fine-sweep samples (finer = sharper peak)
        self.backlash = float(backlash)
        # Let the rig physically settle after each move, then score a fresh, sharp
        # frame instead of one that was still exposing while the rail was moving.
        self.settle_ms = int(settle_ms)   # pause after a move before measuring
        self.flush_frames = int(flush_frames)  # stale frames to discard post-move
        self.samples = int(samples)       # frames averaged per focus measurement
        self._running = False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        started_video = False
        try:
            # Drive our OWN video stream for autofocus, so focus evaluation no longer
            # depends on the GUI live view being active. Frames are pulled and scored
            # exactly the way the original autofocus did (capture_video_frame center
            # crop -> tenengrad_masked). The GUI live view is paused by the caller, so
            # we are the sole reader of the stream here.
            try:
                self.camera.start_video()
                started_video = True
            except Exception as e:
                self.error.emit(f"start_video failed: {e}")
                return

            # === Stage 1: Coarse sweep ===
            self.progress.emit("Starting coarse sweep...")
            coarse_positions, coarse_scores = self._sweep_range(self.coarse_steps, "coarse")

            if not self._running or not coarse_scores:
                self.cancelled.emit()
                return

            best_coarse = coarse_positions[int(np.argmax(coarse_scores))]
            self.progress.emit(f"Coarse peak at {best_coarse:.1f} mm")

            # === Stage 2: Fine sweep ===
            # Original behavior: sweep the fine window high->low so every fine point
            # is reached by a descending ("back") move. Combined with the from-above
            # final positioning below, this keeps the whole fine stage in one backlash
            # state, so the Gaussian peak matches where the lens actually lands.
            self.progress.emit("Starting fine sweep...")
            half = self.fine_window / 2.0
            arr = np.arange(best_coarse - half,
                            best_coarse + half + self.fine_step / 2, self.fine_step)
            arr = np.round(arr, 2)  # avoid float drift in commanded positions
            fine_steps = arr[(arr >= RAIL_MIN) & (arr <= RAIL_MAX)]
            fine_steps = fine_steps[::-1]  # high -> low

            # Approach the top of the fine window from above too, so the first fine
            # measurement shares the same backlash state as the rest of the sweep.
            if len(fine_steps):
                self._move_rail_from_above(float(fine_steps[0]))

            fine_positions, fine_scores = self._sweep_range(fine_steps, "fine")

            if not self._running or not fine_scores:
                self.cancelled.emit()
                return

            # === Stage 3: Fit and move ===
            self.progress.emit("Fitting focus peak...")
            best_fine = self._fit_peak(np.array(fine_positions, float),
                                       np.array(fine_scores, float))
            best_fine = float(np.clip(best_fine, best_coarse - self.fine_window,
                                      best_coarse + self.fine_window))
            best_fine = round(best_fine, 1)

            self.progress.emit(f"Moving to focus point: {best_fine:.1f} mm")
            self._move_rail_from_above(best_fine)

            if self._running:
                self.finished_focus.emit(self.current_pos)
            else:
                self.cancelled.emit()

        except Exception as e:
            self.error.emit(str(e))
        finally:
            # Always release the stream we started, so the GUI live view can resume.
            if started_video:
                self.camera.stop_video()

    def _sweep_range(self, positions, label="sweep"):
        """Sweep through positions and collect focus scores."""
        pos_list, score_list = [], []

        for i, pos in enumerate(positions):
            if not self._running:
                break

            self._move_rail(pos)
            self.progress.emit(f"{label.capitalize()} sweep {i + 1}/{len(positions)}: {pos:.1f} mm")

            # Give the rail/lens time to physically settle before measuring.
            if self.settle_ms > 0:
                self.msleep(self.settle_ms)

            try:
                score = self._measure_focus()
                pos_list.append(float(pos))
                score_list.append(score)

            except Exception as e:
                self.progress.emit(f"Capture error: {e}")
                continue

            # Early exit: if we've peaked and are now clearly declining
            if len(score_list) >= 3:
                if score_list[-1] < score_list[-2] < score_list[-3]:
                    self.progress.emit("Peak detected, stopping sweep early")
                    break

        return pos_list, score_list

    def _measure_focus(self):
        """Score focus at the current (settled) position.

        First discards a few frames so any exposure that began while the rail was
        still moving is thrown away, then averages the focus score over several
        fresh frames to suppress per-frame noise. Both improve peak localization,
        which is what determines final sharpness.
        """
        # Drop stale / in-motion frames.
        for _ in range(max(0, self.flush_frames)):
            if not self._running:
                break
            try:
                self.camera_tab.capture_binned_for_focus()
            except Exception:
                pass

        # Average the focus score over a few fresh frames.
        scores = []
        for _ in range(max(1, self.samples)):
            if not self._running:
                break
            img = self.camera_tab.capture_binned_for_focus()
            s = self.camera_tab.tenengrad_masked(img)
            scores.append(float(s) if hasattr(s, "item") else float(s))

        if not scores:
            raise RuntimeError("no frames captured")
        return float(np.mean(scores))

    def _move_rail(self, target_pos):
        """Move rail to target position."""
        target_pos = float(target_pos)
        if not (RAIL_MIN <= target_pos <= RAIL_MAX):
            return

        d = target_pos - self.current_pos
        if d > 0:
            self.rail.go(d)
            self.rail.wait_until_done("forward")
        elif d < 0:
            self.rail.back(-d)
            self.rail.wait_until_done("backward")

        self.current_pos = target_pos
        self.moved.emit(self.current_pos)

    def _move_rail_from_above(self, target_pos):
        """Reach target_pos by always descending onto it (overshoot, then back down),
        so backlash never offsets the landing. Overshoot exceeds the rail backlash."""
        target_pos = float(target_pos)
        if not (RAIL_MIN <= target_pos <= RAIL_MAX):
            return
        overshoot = max(1.0, self.backlash * 3)
        above = min(RAIL_MAX, target_pos + overshoot)
        if self.current_pos < above:
            self._move_rail(above)          # rise above the target
        self._move_rail(target_pos)         # descend onto it ("back" move)

    @staticmethod
    def _gaussian(x, a, x0, sigma):
        return a * np.exp(-(x - x0) ** 2 / (2 * sigma ** 2))

    def _fit_peak(self, positions, scores):
        """Fit Gaussian to find focus peak."""
        if len(scores) == 0:
            raise ValueError("No scores to fit")

        p0 = [max(scores), positions[int(np.argmax(scores))], 3.0]
        try:
            popt, _ = curve_fit(self._gaussian, positions, scores, p0=p0)
            return popt[1]
        except (RuntimeError, ValueError) as e:
            self.progress.emit(f"Gaussian fit failed: {e}, using measured peak")
            return positions[int(np.argmax(scores))]


class ClickableLabel(QLabel):
    roi_selected = pyqtSignal(QRect)

    def __init__(self):
        super().__init__()
        self._start = None
        self._rect = None

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._start = e.pos()
            self._rect = None

    def mouseMoveEvent(self, e):
        if self._start:
            self._rect = QRect(self._start, e.pos()).normalized()
            self.update()

    def mouseReleaseEvent(self, e):
        if self._start and self._rect:
            self.roi_selected.emit(self._rect)
            self._start = None

    def paintEvent(self, e):
        super().paintEvent(e)
        if self._rect:
            p = QPainter(self)
            p.setPen(QPen(Qt.green, 2, Qt.DashLine))
            p.drawRect(self._rect)


# =============================================================================
# Live preview popup window
# =============================================================================
class LivePreviewWindow(QWidget):
    """Standalone top-level window: live camera feed + recent captures strip."""
    closed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("Live Preview")
        self.setMinimumSize(700, 620)

        layout = QVBoxLayout(self)

        self.view = ClickableLabel()  # <-- changed
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setStyleSheet("background-color:#111; color:#888;")
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view.setText("Waiting for frames...")

        # Clear ROI button
        self.btn_clear_roi = QPushButton("Clear Focus ROI")
        roi_row = QHBoxLayout()
        roi_row.addStretch(1)
        roi_row.addWidget(self.btn_clear_roi)

        self.fps_label = QLabel("FPS: -")
        layout.addWidget(self.view, 1)
        layout.addLayout(roi_row)
        layout.addWidget(self.fps_label)

        # Recent captures strip
        recent = QGroupBox("Recent Captures")
        recent_layout = QVBoxLayout(recent)
        self.recent_scroll = QScrollArea()
        self.recent_scroll.setWidgetResizable(True)
        self.recent_scroll.setFixedHeight(150)
        self.recent_container = QWidget()
        self.recent_container_layout = QHBoxLayout(self.recent_container)
        self.recent_container_layout.setContentsMargins(0, 0, 0, 0)
        self.recent_container_layout.setSpacing(5)
        self.recent_scroll.setWidget(self.recent_container)
        recent_layout.addWidget(self.recent_scroll)
        layout.addWidget(recent)

    def closeEvent(self, ev):
        self.closed.emit()
        super().closeEvent(ev)


# =============================================================================
# Single-image viewer popup window
# =============================================================================
class ImageViewerWindow(QWidget):
    """Standalone top-level window showing one capture at full resolution."""

    def __init__(self, path, parent=None):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle(os.path.basename(path))
        self.setMinimumSize(480, 360)
        self._pix = QPixmap(path)

        layout = QVBoxLayout(self)
        self.view = QLabel()
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setStyleSheet("background-color:#111; color:#888;")
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.view, 1)

        if not self._pix.isNull():
            self.resize(min(self._pix.width(), 1000), min(self._pix.height(), 800))
        self._rescale()

    def resizeEvent(self, ev):
        self._rescale()
        super().resizeEvent(ev)

    def _rescale(self):
        if self._pix.isNull():
            self.view.setText("Could not load image")
            return
        self.view.setPixmap(self._pix.scaled(self.view.size(),
                                             Qt.KeepAspectRatio,
                                             Qt.SmoothTransformation))


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
        self._build_color_offsets(layout)

    def _build_connection(self, parent_layout):
        group = QGroupBox("Rail Connection")
        layout = QHBoxLayout(group)
        self.port_combo = QComboBox()
        self.chk_show_all_ports = QCheckBox("Show all")
        self.chk_show_all_ports.setToolTip("Show non-USB serial ports too")
        self.chk_show_all_ports.toggled.connect(self._refresh_ports)
        self._refresh_ports()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)
        self.btn_save = QPushButton("Save")
        self.btn_save.clicked.connect(self._save_config)
        layout.addWidget(QLabel("Port:"))
        layout.addWidget(self.port_combo, 1)
        layout.addWidget(self.chk_show_all_ports)
        layout.addWidget(self.btn_refresh)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.btn_save)
        parent_layout.addWidget(group)

    def _refresh_ports(self):
        self.port_combo.clear()
        for device, label in list_serial_ports(self.chk_show_all_ports.isChecked()):
            self.port_combo.addItem(label, device)

    def _build_tabs(self, parent_layout):
        self.tabs = QTabWidget()

        # --- Distance ---
        dist = QWidget(); dl = QGridLayout(dist)
        self.dist_total = QLineEdit("10.0")
        self.dist_step = QLineEdit("0.1")
        self.dist_total_steps = QLabel("Total steps: 100")
        self.current_pos_qline = QLineEdit(f"{self.current_pos:.3f}"); self.current_pos_qline.setReadOnly(True)
        dl.addWidget(QLabel("Total distance (mm):"), 0, 0); dl.addWidget(self.dist_total, 0, 1)
        dl.addWidget(QLabel("Step length (mm):"), 1, 0);    dl.addWidget(self.dist_step, 1, 1)
        dl.addWidget(self.dist_total_steps, 2, 0, 1, 2)
        dl.addWidget(QLabel("Position: "), 3, 0);    dl.addWidget(self.current_pos_qline, 3, 1)
        self.zero_btn = QPushButton("Zero")
        self.zero_btn.clicked.connect(self._zero)
        dl.addWidget(self.zero_btn, 3, 2)

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

    def _build_color_offsets(self, parent_layout):
        group = QGroupBox("Color Offset Parameters")
        layout = QGridLayout(group)
        self.red_offset = QLineEdit("0")
        self.green_offset = QLineEdit("0")
        self.blue_offset = QLineEdit("0")
        self.blue_offset = QLineEdit("0")
        self.uv_offset = QLineEdit("0")
        self.infrared_offset = QLineEdit("0")

        layout.addWidget(QLabel("Focus point (green)"), 0, 0)
        layout.addWidget(self.green_offset, 0, 1)
        self.green_btn = QPushButton("Set")
        self.green_btn.clicked.connect(lambda: self.go_to_point(
            float(self.green_offset.text())))
        layout.addWidget(self.green_btn, 0, 3)

        layout.addWidget(QLabel("+ red offset:"), 1, 0)
        layout.addWidget(self.red_offset, 1, 1)
        self.red_btn = QPushButton("Set")
        self.red_btn.clicked.connect(lambda: self.go_to_point(
            float(self.green_offset.text()), float(self.red_offset.text())))
        layout.addWidget(self.red_btn, 1, 3)

        layout.addWidget(QLabel("+ blue Offset:"), 2, 0)
        layout.addWidget(self.blue_offset, 2, 1)
        self.blue_btn = QPushButton("Set")
        self.blue_btn.clicked.connect(lambda: self.go_to_point(
            float(self.green_offset.text()), float(self.blue_offset.text())))
        layout.addWidget(self.blue_btn, 2, 3)

        layout.addWidget(QLabel("+ UV offset:"), 3, 0)
        layout.addWidget(self.uv_offset, 3, 1)
        self.uv_btn = QPushButton("Set")
        self.uv_btn.clicked.connect(lambda: self.go_to_point(
            float(self.green_offset.text()), float(self.uv_offset.text())))
        layout.addWidget(self.uv_btn, 3, 3)

        layout.addWidget(QLabel("+ infrared offset"), 4, 0)
        layout.addWidget(self.infrared_offset, 4, 1)
        self.infrared_btn = QPushButton("Set")
        self.infrared_btn.clicked.connect(lambda: self.go_to_point(
            float(self.green_offset.text()), float(self.infrared_offset.text())))
        layout.addWidget(self.infrared_btn, 4, 3)

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
        port = self.port_combo.currentData()
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
            if d > 0:   self.driver.back(d);
            elif d < 0: self.driver.go(-d);
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
                self.current_pos_qline.setText(f"{self.current_pos:.3f} mm")
            else:
                _, cmd = self.driver.back(step, wait, per_step, interval); self.current_pos -= step
                self.current_pos_qline.setText(f"{self.current_pos:.3f} mm")
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

    def _zero(self):
        self.current_pos = 0
        self.current_pos_qline.setText(f"{self.current_pos:.3f} mm")
        return

    def go_to_point(self, pos, offset=0.0):
        if not self._check_connection(): return
        try:
            goal_pos = pos + offset
            if RAIL_MAX >= goal_pos >= RAIL_MIN:
                d = goal_pos - self.current_pos
                if d > 0:
                    self.driver.go(d)
                    self.driver.wait_until_done("forward")
                elif d < 0:
                    self.driver.back(-d)
                    self.driver.wait_until_done("backward")

                self.current_pos = pos + offset
                self.current_pos_qline.setText(f"{self.current_pos:.3f} mm")
            else:
                QMessageBox.warning(
                    self, "Error", f"Position out of range: {goal_pos} mm"
                )
        except Exception as e:
            QMessageBox.warning(self, "Error going to point", str(e))


# =============================================================================
# Camera tab
# =============================================================================
class CameraTab(QWidget):
    def __init__(self):
        super().__init__()
        self.camera = ZWOCamera()
        self.live_thread = None
        self.live_window = None
        self._viewer_windows = []
        self._last_frame = None
        self._focus_roi = None

        self.snap_thread = None
        self._snap_was_live = False
        self._snap_start = 0.0
        self._snap_expected_ms = 0.0
        self._snap_indeterminate = False

        self._ensure_capture_dir()
        self.recent_capture_paths = []

        root = QHBoxLayout(self)

        left = QVBoxLayout()
        root.addLayout(left, 1)
        root.addStretch(1)

        # ----- Connection -----
        conn = QGroupBox("Camera Connection")
        cl = QGridLayout(conn)
        self.cam_combo = QComboBox()
        self.btn_refresh = QPushButton("Refresh");
        self.btn_refresh.clicked.connect(self._refresh)
        self.btn_connect = QPushButton("Connect");
        self.btn_connect.clicked.connect(self._toggle_connect)
        self.info_label = QLabel("Not connected")
        cl.addWidget(QLabel("Camera:"), 0, 0);
        cl.addWidget(self.cam_combo, 0, 1, 1, 2)
        cl.addWidget(self.btn_refresh, 1, 1);
        cl.addWidget(self.btn_connect, 1, 2)
        cl.addWidget(self.info_label, 2, 0, 1, 3)
        left.addWidget(conn)

        # ----- Exposure / Gain -----
        eg = QGroupBox("Exposure & Gain")
        g = QGridLayout(eg)
        self.exposure_us = QSpinBox();
        self.exposure_us.setRange(32, 600_000_000)
        self.exposure_us.setSuffix(" µs");
        self.exposure_us.setValue(10_000)
        self.exposure_us.valueChanged.connect(self._apply_exposure)
        self.gain = QSpinBox();
        self.gain.setRange(0, 500);
        self.gain.setValue(100)
        self.gain.valueChanged.connect(self._apply_gain)
        g.addWidget(QLabel("Exposure:"), 0, 0);
        g.addWidget(self.exposure_us, 0, 1)
        g.addWidget(QLabel("Gain:"), 1, 0);
        g.addWidget(self.gain, 1, 1)
        left.addWidget(eg)

        # ----- Binning / ROI -----
        bg = QGroupBox("Binning / Image Type")
        bl = QGridLayout(bg)
        self.bin_combo = QComboBox();
        self.bin_combo.addItems(["1", "2", "4"])
        self.type_combo = QComboBox();
        self.type_combo.addItems(["RAW8", "RAW16"])
        self.btn_apply_roi = QPushButton("Apply");
        self.btn_apply_roi.clicked.connect(self._apply_roi)
        bl.addWidget(QLabel("Bin (NxN):"), 0, 0);
        bl.addWidget(self.bin_combo, 0, 1)
        bl.addWidget(QLabel("Image type:"), 1, 0);
        bl.addWidget(self.type_combo, 1, 1)
        bl.addWidget(self.btn_apply_roi, 2, 0, 1, 2)
        left.addWidget(bg)

        # ----- Temperature / Cooler -----
        tg = QGroupBox("Temperature & Cooler")
        tl = QGridLayout(tg)
        self.temp_label = QLabel("Sensor: --- °C")
        self.cooler_chk = QCheckBox("Cooler ON");
        self.cooler_chk.toggled.connect(self._apply_cooler)
        self.target_temp = QDoubleSpinBox();
        self.target_temp.setRange(-40, 40);
        self.target_temp.setValue(0)
        self.target_temp.setSuffix(" °C");
        self.target_temp.valueChanged.connect(self._apply_cooler)
        self.cooler_pwr = QLabel("Power: -")
        tl.addWidget(self.temp_label, 0, 0, 1, 2)
        tl.addWidget(self.cooler_chk, 1, 0)
        tl.addWidget(self.target_temp, 1, 1)
        tl.addWidget(self.cooler_pwr, 2, 0, 1, 2)
        left.addWidget(tg)

        # ----- Capture -----
        lv = QGroupBox("Capture")
        ll = QGridLayout(lv)
        self.btn_live = QPushButton("Start Live View");
        self.btn_live.clicked.connect(self._toggle_live)
        self.btn_snap = QPushButton("Trigger Shutter (Snap)");
        self.btn_snap.clicked.connect(self._snap)
        self.capture_folder_label = QLabel()
        self.capture_folder_label.setWordWrap(True)
        self.capture_folder_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        ll.addWidget(self.btn_live, 0, 0);
        ll.addWidget(self.btn_snap, 0, 1)
        ll.addWidget(QLabel("Saved to:"), 1, 0);
        ll.addWidget(self.capture_folder_label, 1, 1)
        self.btn_show_view = QPushButton("Open Preview Window")
        self.btn_show_view.clicked.connect(self._open_window)
        ll.addWidget(self.btn_show_view, 2, 0, 1, 2)
        self.snap_progress = QProgressBar()
        self.snap_progress.setRange(0, 100)
        self.snap_progress.setTextVisible(True)
        self.snap_progress.setVisible(False)
        ll.addWidget(self.snap_progress, 3, 0, 1, 2)
        left.addWidget(lv)

        left.addStretch(1)

        # Live preview window (separate top-level window)
        self.live_window = LivePreviewWindow(self)
        self.live_window.closed.connect(self._on_live_window_closed)
        self.live_window.view.roi_selected.connect(self._on_roi_selected)
        self.live_window.btn_clear_roi.clicked.connect(self._clear_roi)

        # Timers
        self.temp_timer = QTimer(self);
        self.temp_timer.timeout.connect(self._poll_temp)
        self.temp_timer.start(1000)
        self.snap_timer = QTimer(self);
        self.snap_timer.timeout.connect(self._update_snap_progress)

        self._refresh()
        self.capture_folder_label.setText(self.capture_dir)
        self._load_recent_captures()

    def _on_roi_selected(self, rect):
        if self._last_frame is None:
            return
        lw, lh = self.live_window.view.width(), self.live_window.view.height()
        ih, iw = self._last_frame.shape[:2]
        scale = min(lw / iw, lh / ih)
        disp_w = iw * scale
        disp_h = ih * scale
        ox = (lw - disp_w) / 2
        oy = (lh - disp_h) / 2

        # map label pixels -> full frame pixels
        x = int((rect.x() - ox) / scale)
        y = int((rect.y() - oy) / scale)
        w = int(rect.width() / scale)
        h = int(rect.height() / scale)

        # shift into the coordinate space of the binned crop
        # (capture_binned_for_focus takes the center quarter)
        x -= iw // 4
        y -= ih // 4

        # clamp to the cropped frame size
        crop_w, crop_h = iw // 2, ih // 2
        x = max(0, min(x, crop_w - 1))
        y = max(0, min(y, crop_h - 1))
        w = max(1, min(w, crop_w - x))
        h = max(1, min(h, crop_h - y))

        self._focus_roi = (x, y, w, h)

    def _clear_roi(self):
        self._focus_roi = None
        self.live_window.view._rect = None
        self.live_window.view.update()

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
        if self.live_window is None:
            return
        recent_layout = self.live_window.recent_container_layout
        for i in reversed(range(recent_layout.count())):
            widget = recent_layout.takeAt(i).widget()
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
            recent_layout.addWidget(button)

        recent_layout.addStretch(1)

    def _load_recent_image(self, path):
        # Open the selected capture in its own separate window
        viewer = ImageViewerWindow(path, self)
        viewer.show()
        viewer.raise_()
        viewer.activateWindow()
        # Keep a reference so the window isn't garbage-collected
        self._viewer_windows = [v for v in self._viewer_windows if v.isVisible()]
        self._viewer_windows.append(viewer)

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

    def _open_window(self):
        # Show (and bring to front) the standalone preview / captures window
        self.live_window.show()
        self.live_window.raise_()
        self.live_window.activateWindow()

    def _start_live(self):
        if not self.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera not connected"); return
        self.live_window.view.setText("Waiting for frames...")
        self._open_window()

        self.live_thread = LiveViewThread(self.camera)
        self.live_thread.frame_ready.connect(self._on_frame)
        self.live_thread.warning.connect(self._on_live_warning)
        self.live_thread.error.connect(self._on_live_error)
        self.live_thread.start()
        self.btn_live.setText("Stop Live View")

    def _stop_live(self):
        if self.live_thread:
            self.live_thread.stop()
            self.live_thread = None
        self.btn_live.setText("Start Live View")

    def _on_live_window_closed(self):
        # User closed the preview window directly: stop the live feed too
        if self.live_thread:
            self.live_thread.stop()
            self.live_thread = None
            self.btn_live.setText("Start Live View")

    def _on_live_warning(self, msg):
        # Non-fatal: a frame was dropped and the stream is recovering
        print("live warn:", msg)
        if self.live_window is not None:
            self.live_window.fps_label.setText(msg)

    def _on_live_error(self, msg):
        # Fatal: the stream stopped. Reset UI state so it stays in sync.
        print("live err:", msg)
        self._stop_live()
        if self.live_window is not None:
            self.live_window.view.setText("Live view stopped")
        QMessageBox.warning(self, "Live View",
                            f"Live view stopped:\n{msg}\n\n"
                            f"Tip: very long exposures reduce the frame rate; "
                            f"the timeout now scales with exposure automatically.")

    def _on_frame(self, frame: np.ndarray, fps: float):
        self._last_frame = frame
        if self.live_window is None:
            return
        pix = self._frame_to_pixmap(frame)
        if pix is not None:
            self.live_window.view.setPixmap(
                pix.scaled(self.live_window.view.size(),
                           Qt.KeepAspectRatio, Qt.FastTransformation))
        self.live_window.fps_label.setText(
            f"FPS: {fps:.1f}   shape: {frame.shape}  dtype: {frame.dtype}")

    def _frame_to_pixmap(self, frame: np.ndarray):
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
            return None
        return QPixmap.fromImage(img)

    def _show_frame(self, frame: np.ndarray):
        pix = self._frame_to_pixmap(frame)
        if pix is None:
            return
        # scale to the preview window's label size
        self.live_window.view.setPixmap(pix.scaled(self.live_window.view.size(),
                                                   Qt.KeepAspectRatio,
                                                   Qt.FastTransformation))

    def _snap(self):
        if not self.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera not connected"); return
        if self.snap_thread is not None:
            return  # a capture is already in progress

        # Pause live view during the still exposure; restore it afterwards.
        self._snap_was_live = self.live_thread is not None
        if self._snap_was_live:
            self._stop_live()

        self._open_window()

        # Lock out re-entrancy and pause temp polling so the SDK isn't hit
        # from two threads while the long exposure is running.
        self.btn_snap.setEnabled(False)
        self.btn_live.setEnabled(False)
        self.btn_apply_roi.setEnabled(False)
        self.temp_timer.stop()

        # Determinate % during the (known) exposure, then a continuous busy
        # animation during the unknown sensor-readout / USB-transfer phase.
        self._snap_expected_ms = max(50.0, getattr(self.camera, "last_exposure_us", 10_000) / 1000.0)
        self._snap_start = time.time()
        self._snap_indeterminate = False
        self.snap_progress.setRange(0, 100)
        self.snap_progress.setValue(0)
        self.snap_progress.setFormat("Capturing… %p%")
        self.snap_progress.setVisible(True)
        self.live_window.view.setText("Capturing…")
        self.live_window.fps_label.setText("Capturing…")
        self.snap_timer.start(50)

        self.snap_thread = SnapThread(self.camera)
        self.snap_thread.captured.connect(self._on_snap_done)
        self.snap_thread.error.connect(self._on_snap_error)
        self.snap_thread.start()

    def _update_snap_progress(self):
        elapsed_ms = (time.time() - self._snap_start) * 1000.0
        if self._snap_expected_ms > 0 and elapsed_ms < self._snap_expected_ms:
            # Determinate phase: % of the expected exposure elapsed.
            frac = elapsed_ms / self._snap_expected_ms
            self.snap_progress.setValue(int(frac * 100))
        elif not self._snap_indeterminate:
            # Exposure window elapsed; readout/transfer time is unknown, so show a
            # continuously-animating busy bar instead of freezing at a fixed value.
            self._snap_indeterminate = True
            self.snap_progress.setRange(0, 0)
            self.snap_progress.setFormat("Capturing… (reading out)")

    def _on_snap_done(self, frame: np.ndarray):
        info = None
        try:
            self._show_frame(frame)
            self.live_window.fps_label.setText(
                f"Snapshot  shape: {frame.shape}  dtype: {frame.dtype}")
            saved_path = self._save_capture(frame)
            self.capture_folder_label.setText(self.capture_dir)
            self._load_recent_captures()
            info = f"Captured {frame.shape} ({frame.dtype})\nSaved to:\n{saved_path}"
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Saving capture failed: {e}")
        self._finish_snap()
        if info:
            QMessageBox.information(self, "Snap", info)

    def _on_snap_error(self, msg: str):
        self.live_window.view.setText("Capture failed")
        self._finish_snap()
        QMessageBox.warning(self, "Error", f"Snap failed: {msg}")

    def _finish_snap(self):
        self.snap_timer.stop()
        self._snap_indeterminate = False
        self.snap_progress.setRange(0, 100)
        self.snap_progress.setValue(100)
        self.snap_progress.setFormat("%p%")
        self.snap_progress.setVisible(False)
        self.btn_snap.setEnabled(True)
        self.btn_live.setEnabled(True)
        self.btn_apply_roi.setEnabled(True)
        if not self.temp_timer.isActive():
            self.temp_timer.start(1000)
        if self.snap_thread is not None:
            self.snap_thread.wait(2000)
            self.snap_thread = None
        if self._snap_was_live:
            self._snap_was_live = False
            self._start_live()

    def closeEvent(self, ev):
        self._stop_live()
        self.snap_timer.stop()
        if self.snap_thread is not None:
            self.snap_thread.wait(5000)
            self.snap_thread = None
        if self.live_window is not None:
            self.live_window.close()
        for v in self._viewer_windows:
            v.close()
        self.camera.disconnect()
        super().closeEvent(ev)

    def capture_binned_for_focus(self):
        """
        Capture a frame optimized for autofocus scoring.
        Returns uint8 center ROI.
        """
        if not self.camera.is_connected():
            raise RuntimeError("Camera not connected")

        exp_us = getattr(self.camera, "last_exposure_us", 10_000)
        timeout_ms = int(max(2000, exp_us / 1000.0 * 2 + 500))
        frame = self.camera.capture_video_frame(timeout_ms=timeout_ms)

        if frame.dtype == np.uint16:
            frame_8 = (frame / 256).astype(np.uint8)
        else:
            frame_8 = frame.astype(np.uint8)

        h, w = frame_8.shape[:2]

        return frame_8[
            h // 4: 3 * h // 4,
            w // 4: 3 * w // 4
        ]

    def tenengrad_masked(self, img, roi=None):
        if roi is not None:
            x, y, w, h = roi
            # clamp to actual image size
            ih, iw = img.shape[:2]
            x = max(0, min(x, iw - 1))
            y = max(0, min(y, ih - 1))
            w = max(1, min(w, iw - x))
            h = max(1, min(h, ih - y))
            cropped = img[y:y + h, x:x + w]
            if cropped.size == 0:
                print("Warning: ROI crop is empty, falling back to full image")
            else:
                img = cropped

        mask = img < 240
        gx = cv2.Sobel(img, cv2.CV_64F, 1, 0)
        gy = cv2.Sobel(img, cv2.CV_64F, 0, 1)
        return np.mean((gx ** 2 + gy ** 2)[mask])


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
        self.chk_show_all_ports = QCheckBox("Show all")
        self.chk_show_all_ports.setToolTip("Show non-USB serial ports too")
        self.chk_show_all_ports.toggled.connect(self._refresh)
        self.btn_refresh = QPushButton("Refresh"); self.btn_refresh.clicked.connect(self._refresh)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.clicked.connect(self._toggle_connect)
        cl.addWidget(QLabel("Port:")); cl.addWidget(self.port_combo, 1)
        cl.addWidget(self.chk_show_all_ports)
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
        for device, label in list_serial_ports(self.chk_show_all_ports.isChecked()):
            self.port_combo.addItem(label, device)

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

        self.af_thread = None
        self._af_resume_live = False

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
        self.autofocus_btn = QPushButton("Auto Focus"); self.autofocus_btn.clicked.connect(self._toggle_autofocus)
        layout.addWidget(self.autofocus_btn)



        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        self.setCentralWidget(scroll)

    def closeEvent(self, ev):
        if self.af_thread is not None:
            self.af_thread.stop()
            self.af_thread.wait(5000)
            self.af_thread = None
        self.camera_tab.closeEvent(ev)
        self.stm32_tab.closeEvent(ev)
        super().closeEvent(ev)

    # ---- auto focus (runs on a worker thread; pauses live view) ----
    def _toggle_autofocus(self):
        if self.af_thread is not None:
            self.af_thread.stop()
            self.autofocus_btn.setEnabled(False)
            self.autofocus_btn.setText("Cancelling…")
            return
        self._start_autofocus()

    def _start_autofocus(self):
        if not (self.rail_tab.driver and self.rail_tab.driver.is_connected()):
            QMessageBox.warning(self, "Error", "Rail is not connected"); return
        if not self.camera_tab.camera.is_connected():
            QMessageBox.warning(self, "Error", "Camera is not connected"); return

        # Pause live view so the AF worker is the only camera reader.
        self._af_resume_live = self.camera_tab.live_thread is not None
        if self._af_resume_live:
            self.camera_tab._stop_live()

        # Lock out the rail/camera UI while the worker drives the rail.
        self.rail_tab.setEnabled(False)
        self.camera_tab.setEnabled(False)
        self.autofocus_btn.setText("Cancel Auto Focus")

        coarse_steps = np.arange(RAIL_MIN, RAIL_MAX + 1, 10)
        self.af_thread = AutoFocusThread(self.camera_tab.camera, self.rail_tab.driver,
                                         self.camera_tab, self.rail_tab.current_pos, coarse_steps,
                                         backlash=self.rail_tab.backlash_val)
        self.af_thread.moved.connect(self._af_moved)
        self.af_thread.progress.connect(lambda m: print("autofocus:", m))
        self.af_thread.finished_focus.connect(self._af_done)
        self.af_thread.cancelled.connect(self._af_cancelled)
        self.af_thread.error.connect(self._af_error)
        self.af_thread.start()

    def _af_moved(self, pos):
        # Keep the rail tab's bookkeeping in sync (GUI thread).
        self.rail_tab.current_pos = pos
        self.rail_tab.current_pos_qline.setText(f"{pos:.3f} mm")

    def _af_done(self, best):
        self._af_cleanup()
        QMessageBox.information(self, "Auto Focus", f"Focused at {best:.2f} mm")

    def _af_cancelled(self):
        self._af_cleanup()

    def _af_error(self, msg):
        self._af_cleanup()
        QMessageBox.warning(self, "Auto Focus", f"Auto focus failed:\n{msg}")

    def _af_cleanup(self):
        if self.af_thread is not None:
            self.af_thread.wait(2000)
            self.af_thread = None
        self.rail_tab.setEnabled(True)
        self.camera_tab.setEnabled(True)
        self.autofocus_btn.setEnabled(True)
        self.autofocus_btn.setText("Auto Focus")
        if self._af_resume_live:
            self._af_resume_live = False
            self.camera_tab._start_live()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())