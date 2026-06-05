"""
Standalone PyQt5 control panel for a WLED LED strip driven by an ESP32 over USB.

A lightweight local replacement for the WLED web UI: power, brightness, color,
effects/palettes, speed/intensity, presets, a segment "which LEDs are on" range
(plus individual-pixel entry), and a raw JSON console for debugging.

Run:  python wled_ui.py
"""

import sys
import json

import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QLabel, QComboBox, QPushButton, QCheckBox, QLineEdit, QSpinBox,
    QSlider, QTextEdit, QColorDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

from ESP32_driver import WLEDDriver
from wled_data import EFFECTS, PALETTES, EFFECTS_SOURCE_VERSION


def list_serial_ports(show_all=False):
    """[(device, label)] for serial ports. USB-only (has a VID) unless show_all."""
    out = []
    for p in serial.tools.list_ports.comports():
        if not show_all and p.vid is None:
            continue
        desc = (p.description or p.product or "").strip()
        label = f"{p.device}  ({desc})" if desc and desc.lower() != "n/a" else p.device
        out.append((p.device, label))
    return out


class WLEDWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WLED Serial Control")
        self.driver = None
        self.led_count = 30   # default; auto-set on connect, editable in the UI
        self.cur_color = QColor(255, 255, 255)

        central = QWidget()
        root = QVBoxLayout(central)
        self._build_connection(root)
        self._build_power_brightness(root)
        self._build_color(root)
        self._build_effects(root)
        self._build_presets(root)
        self._build_range(root)
        self._build_console(root)
        root.addStretch(1)
        self.setCentralWidget(central)

        # Debounce slewy sliders so a drag doesn't flood the serial link.
        self._pending = {}
        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(80)
        self._debounce.timeout.connect(self._flush_sliders)

        # Drain RX into the console periodically.
        self.rx_timer = QTimer(self)
        self.rx_timer.timeout.connect(self._drain_rx)
        self.rx_timer.start(100)

        self._set_controls_enabled(False)
        self._refresh_ports()

    # ---------- UI builders ----------
    def _build_connection(self, parent):
        grp = QGroupBox("Connection")
        h = QHBoxLayout(grp)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        for b in ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]:
            self.baud_combo.addItem(b)
        self.baud_combo.setCurrentText("115200")
        self.chk_show_all = QCheckBox("Show all")
        self.chk_show_all.setToolTip("Show non-USB serial ports too")
        self.chk_show_all.toggled.connect(self._refresh_ports)
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connect)
        h.addWidget(QLabel("Port:")); h.addWidget(self.port_combo, 1)
        h.addWidget(self.chk_show_all)
        h.addWidget(QLabel("Baud:")); h.addWidget(self.baud_combo)
        h.addWidget(self.btn_refresh); h.addWidget(self.btn_connect)
        parent.addWidget(grp)

    def _build_power_brightness(self, parent):
        grp = QGroupBox("Power & Brightness")
        g = QGridLayout(grp)
        self.chk_power = QCheckBox("Power")
        self.chk_power.toggled.connect(lambda on: self._send(lambda: self.driver.set_power(on)))
        self.bri_slider = QSlider(Qt.Horizontal); self.bri_slider.setRange(0, 255); self.bri_slider.setValue(128)
        self.bri_label = QLabel("128")
        self.bri_slider.valueChanged.connect(self._on_bri_changed)
        self.bri_slider.sliderReleased.connect(self._flush_sliders)  # send immediately on release
        g.addWidget(self.chk_power, 0, 0)
        g.addWidget(QLabel("Brightness:"), 1, 0)
        g.addWidget(self.bri_slider, 1, 1)
        g.addWidget(self.bri_label, 1, 2)
        parent.addWidget(grp)

    def _build_color(self, parent):
        grp = QGroupBox("Primary Color")
        h = QHBoxLayout(grp)
        self.color_swatch = QLabel()
        self.color_swatch.setFixedSize(60, 24)
        self.btn_pick_color = QPushButton("Pick Color…")
        self.btn_pick_color.clicked.connect(self._pick_color)
        h.addWidget(self.color_swatch)
        h.addWidget(self.btn_pick_color)
        h.addStretch(1)
        parent.addWidget(grp)
        self._update_color_swatch()

    def _build_effects(self, parent):
        grp = QGroupBox("Effects & Palettes")
        g = QGridLayout(grp)
        self.fx_combo = QComboBox(); self.fx_combo.addItems(EFFECTS)
        self.fx_combo.currentIndexChanged.connect(
            lambda i: self._send(lambda: self.driver.set_effect(i)))
        self.pal_combo = QComboBox(); self.pal_combo.addItems(PALETTES)
        self.pal_combo.currentIndexChanged.connect(
            lambda i: self._send(lambda: self.driver.set_palette(i)))
        self.sx_slider = QSlider(Qt.Horizontal); self.sx_slider.setRange(0, 255); self.sx_slider.setValue(128)
        self.sx_label = QLabel("128")
        self.sx_slider.valueChanged.connect(self._on_sx_changed)
        self.sx_slider.sliderReleased.connect(self._flush_sliders)
        self.ix_slider = QSlider(Qt.Horizontal); self.ix_slider.setRange(0, 255); self.ix_slider.setValue(128)
        self.ix_label = QLabel("128")
        self.ix_slider.valueChanged.connect(self._on_ix_changed)
        self.ix_slider.sliderReleased.connect(self._flush_sliders)
        g.addWidget(QLabel("Effect:"), 0, 0);  g.addWidget(self.fx_combo, 0, 1, 1, 2)
        g.addWidget(QLabel("Palette:"), 1, 0); g.addWidget(self.pal_combo, 1, 1, 1, 2)
        g.addWidget(QLabel("Speed:"), 2, 0);   g.addWidget(self.sx_slider, 2, 1); g.addWidget(self.sx_label, 2, 2)
        g.addWidget(QLabel("Intensity:"), 3, 0); g.addWidget(self.ix_slider, 3, 1); g.addWidget(self.ix_label, 3, 2)
        parent.addWidget(grp)

    def _build_presets(self, parent):
        grp = QGroupBox("Presets")
        h = QHBoxLayout(grp)
        self.preset_spin = QSpinBox(); self.preset_spin.setRange(1, 250); self.preset_spin.setValue(1)
        self.btn_recall = QPushButton("Recall"); self.btn_recall.clicked.connect(self._recall_preset)
        self.btn_save = QPushButton("Save"); self.btn_save.clicked.connect(self._save_preset)
        h.addWidget(QLabel("Preset ID:")); h.addWidget(self.preset_spin)
        h.addWidget(self.btn_recall); h.addWidget(self.btn_save)
        h.addStretch(1)
        parent.addWidget(grp)

    def _build_range(self, parent):
        grp = QGroupBox("Which LEDs are on")
        g = QGridLayout(grp)
        # LED count: auto-filled on connect, but editable — serial state sync
        # isn't always available, and everything here depends on knowing it.
        self.led_count_spin = QSpinBox(); self.led_count_spin.setRange(1, 1200)
        self.led_count_spin.setValue(self.led_count)   # set BEFORE connecting the signal
        self.led_count_spin.setToolTip("Number of LEDs on the strip (auto-detected on connect; fix if wrong)")
        self.range_start = QSpinBox(); self.range_start.setRange(0, self.led_count)
        self.range_stop = QSpinBox(); self.range_stop.setRange(0, self.led_count); self.range_stop.setValue(self.led_count)
        self.led_count_spin.valueChanged.connect(self._on_led_count_changed)
        self.btn_apply_range = QPushButton("Show only range (rest off)")
        self.btn_apply_range.clicked.connect(self._apply_range)
        self.ind_edit = QLineEdit(); self.ind_edit.setPlaceholderText("Individual LEDs, e.g. 0,3,5-9")
        self.btn_apply_ind = QPushButton("Show only these (rest off)")
        self.btn_apply_ind.clicked.connect(self._apply_individual)
        self.btn_reset_all = QPushButton("All On / Reset")
        self.btn_reset_all.setToolTip("Unfreeze the strip and light it solid white so effects can run again")
        self.btn_reset_all.clicked.connect(self._reset_all)
        g.addWidget(QLabel("LED count:"), 0, 0); g.addWidget(self.led_count_spin, 0, 1)
        g.addWidget(QLabel("Start:"), 1, 0); g.addWidget(self.range_start, 1, 1)
        g.addWidget(QLabel("Stop (exclusive):"), 1, 2); g.addWidget(self.range_stop, 1, 3)
        g.addWidget(self.btn_apply_range, 1, 4)
        g.addWidget(self.ind_edit, 2, 0, 1, 4)
        g.addWidget(self.btn_apply_ind, 2, 4)
        g.addWidget(self.btn_reset_all, 3, 4)
        parent.addWidget(grp)

    def _build_console(self, parent):
        grp = QGroupBox("Raw JSON Console")
        v = QVBoxLayout(grp)
        row = QHBoxLayout()
        self.json_edit = QLineEdit()
        self.json_edit.setPlaceholderText('{"on":true,"bri":128}')
        self.json_edit.returnPressed.connect(self._send_json_clicked)
        self.btn_send_json = QPushButton("Send"); self.btn_send_json.clicked.connect(self._send_json_clicked)
        self.btn_query = QPushButton("Query State"); self.btn_query.clicked.connect(self._query_state)
        row.addWidget(self.json_edit, 1); row.addWidget(self.btn_send_json); row.addWidget(self.btn_query)
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setStyleSheet("background:#101010;color:#cfc;font-family:monospace;")
        btns = QHBoxLayout()
        self.btn_clear = QPushButton("Clear"); self.btn_clear.clicked.connect(self.console.clear)
        btns.addStretch(1); btns.addWidget(self.btn_clear)
        v.addLayout(row); v.addWidget(self.console, 1); v.addLayout(btns)
        parent.addWidget(grp, 1)

    # ---------- connection ----------
    def _refresh_ports(self):
        self.port_combo.clear()
        for device, label in list_serial_ports(self.chk_show_all.isChecked()):
            self.port_combo.addItem(label, device)

    def _toggle_connect(self):
        if self.driver and self.driver.is_connected():
            self.driver.disconnect(); self.driver = None
            self.btn_connect.setText("Connect")
            self._set_controls_enabled(False)
            self._log("-- disconnected --")
            return
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Error", "No port selected"); return
        baud = int(self.baud_combo.currentText())
        try:
            self.driver = WLEDDriver(port, baudrate=baud)
            self.driver.connect()
            self.btn_connect.setText("Disconnect")
            self._log(f"-- connected {port} @ {baud} --")
            self._set_controls_enabled(True)
            self._sync_from_device()
        except Exception as e:
            self.driver = None
            QMessageBox.warning(self, "Error", f"Connect failed: {e}")

    def _sync_from_device(self):
        state, info = self.driver.get_state(timeout=1.5)
        if not state:
            self._log("!! no state reply — check baud, or is LED output on GPIO1/GPIO3 (UART)?")
            return
        if info and isinstance(info.get("leds"), dict):
            cnt = int(info["leds"].get("count", 0))
            if cnt > 0:
                self.led_count_spin.setValue(cnt)  # fires _on_led_count_changed -> led_count + maxima

        seg0 = (state.get("seg") or [{}])[0]
        self._set_blocked(self.chk_power, bool(state.get("on", False)))
        self._set_blocked(self.bri_slider, int(state.get("bri", 128)))
        self.bri_label.setText(str(self.bri_slider.value()))
        self._set_combo_blocked(self.fx_combo, int(seg0.get("fx", 0)))
        self._set_combo_blocked(self.pal_combo, int(seg0.get("pal", 0)))
        self._set_blocked(self.sx_slider, int(seg0.get("sx", 128)))
        self.sx_label.setText(str(self.sx_slider.value()))
        self._set_blocked(self.ix_slider, int(seg0.get("ix", 128)))
        self.ix_label.setText(str(self.ix_slider.value()))
        self._set_blocked(self.range_start, int(seg0.get("start", 0)))
        self._set_blocked(self.range_stop, int(seg0.get("stop", self.led_count)))
        ps = int(state.get("ps", -1))
        if ps > 0:
            self._set_blocked(self.preset_spin, ps)

        col = seg0.get("col") or [[255, 255, 255]]
        rgb = (list(col[0]) + [0, 0, 0])[:3]
        self.cur_color = QColor(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        self._update_color_swatch()

        ver = info.get("ver", "?") if info else "?"
        self._log(f"synced: leds={self.led_count} ver={ver}")
        if info and not str(ver).startswith(EFFECTS_SOURCE_VERSION):
            self._log(f"!! firmware {ver} != effect list {EFFECTS_SOURCE_VERSION}.x — "
                      f"effect/palette indices may not match names.")

    # ---------- control handlers ----------
    def _on_bri_changed(self, v):
        self.bri_label.setText(str(v))
        self._pending["bri"] = v; self._debounce.start()

    def _on_sx_changed(self, v):
        self.sx_label.setText(str(v))
        self._pending["sx"] = v; self._debounce.start()

    def _on_ix_changed(self, v):
        self.ix_label.setText(str(v))
        self._pending["ix"] = v; self._debounce.start()

    def _on_led_count_changed(self, n):
        self.led_count = int(n)
        self.range_start.setMaximum(self.led_count)
        self.range_stop.setMaximum(self.led_count)

    def _flush_sliders(self):
        if not self.driver or not self.driver.is_connected():
            self._pending = {}
            return
        p = self._pending; self._pending = {}
        try:
            if "bri" in p:
                self.driver.set_brightness(p["bri"]); self._log(f"TX bri={p['bri']}")
            if "sx" in p:
                self.driver.set_speed(p["sx"]); self._log(f"TX sx={p['sx']}")
            if "ix" in p:
                self.driver.set_intensity(p["ix"]); self._log(f"TX ix={p['ix']}")
        except Exception as e:
            self._log(f"!! send err: {e}")

    def _pick_color(self):
        c = QColorDialog.getColor(self.cur_color, self, "Primary Color")
        if c.isValid():
            self.cur_color = c
            self._update_color_swatch()
            self._send(lambda: self.driver.set_color(c.red(), c.green(), c.blue()))

    def _update_color_swatch(self):
        c = self.cur_color
        self.color_swatch.setStyleSheet(
            f"background-color: rgb({c.red()},{c.green()},{c.blue()}); border:1px solid #888;")

    def _recall_preset(self):
        n = self.preset_spin.value()
        self._send(lambda: self.driver.recall_preset(n))
        # Reflect the recalled state in the controls.
        QTimer.singleShot(150, self._sync_from_device)

    def _save_preset(self):
        n = self.preset_spin.value()
        if QMessageBox.question(self, "Save Preset",
                                f"Save current state to preset {n}?") != QMessageBox.Yes:
            return
        self._send(lambda: self.driver.save_preset(n))
        self._log(f"saved preset {n}")

    def _apply_range(self):
        a, b = self.range_start.value(), self.range_stop.value()
        if b <= a:
            QMessageBox.warning(self, "Range", "Stop must be greater than Start."); return
        self._show_only(range(a, b))

    def _apply_individual(self):
        self._show_only(self._parse_indices(self.ind_edit.text()))

    def _show_only(self, indices):
        """Light ONLY the given LED indices (current color); turn the rest off."""
        if not self.driver or not self.driver.is_connected():
            QMessageBox.warning(self, "Error", "Not connected"); return
        indices = [i for i in indices if 0 <= i < self.led_count]
        if not indices:
            QMessageBox.warning(self, "LEDs",
                                f"No valid LED indices for a {self.led_count}-LED strip "
                                f"(valid: 0..{self.led_count - 1}).\n"
                                f"If your strip is longer, raise 'LED count'.")
            return
        c = self.cur_color
        pairs = [(i, (c.red(), c.green(), c.blue())) for i in indices]
        try:
            # Black out the whole strip (this freezes the segment), then write
            # only the chosen LEDs on top — no effect repaints over them.
            self.driver.clear(self.led_count)
            for k in range(0, len(pairs), 100):
                self.driver.show_only(pairs[k:k + 100])
            self._log(f"only {len(indices)} LED(s) on")
        except Exception as e:
            self._log(f"!! {e}")

    def _reset_all(self):
        self._send(lambda: self.driver.reset_all(self.led_count))
        self._log("reset: full strip on (white)")

    def _parse_indices(self, text):
        out = []
        for tok in text.replace(" ", "").split(","):
            if not tok:
                continue
            try:
                if "-" in tok:
                    a, b = tok.split("-", 1)
                    a, b = int(a), int(b)
                    out += list(range(min(a, b), max(a, b) + 1))
                else:
                    out.append(int(tok))
            except ValueError:
                continue
        out = [i for i in out if 0 <= i < self.led_count]
        return sorted(set(out))

    # ---------- console ----------
    def _send_json_clicked(self):
        text = self.json_edit.text().strip()
        if not text:
            return
        try:
            obj = json.loads(text)
        except ValueError as e:
            self._log(f"!! invalid JSON: {e}")
            return
        if not isinstance(obj, dict):
            self._log("!! JSON must be an object, e.g. {\"on\":true}")
            return
        self._send(lambda: self.driver.send_json(obj))
        self._log(f"TX: {json.dumps(obj, separators=(',', ':'))}")
        self.json_edit.clear()

    def _query_state(self):
        if not self.driver or not self.driver.is_connected():
            QMessageBox.warning(self, "Error", "Not connected"); return
        state, info = self.driver.get_state(timeout=1.5)
        if state is None and info is None:
            self._log("!! query: no reply")
            return
        self._log("STATE: " + json.dumps(state))
        self._sync_from_device()

    def _drain_rx(self):
        if not self.driver or not self.driver.is_connected():
            return
        for ts, line in self.driver.pop_lines():
            self._log(f"RX: {line}")

    def _log(self, text):
        self.console.append(text)

    # ---------- helpers ----------
    def _send(self, fn):
        if not self.driver or not self.driver.is_connected():
            QMessageBox.warning(self, "Error", "Not connected"); return
        try:
            fn()
        except Exception as e:
            self._log(f"!! {e}")

    @staticmethod
    def _set_blocked(widget, value):
        widget.blockSignals(True)
        if isinstance(widget, QCheckBox):
            widget.setChecked(bool(value))
        else:
            widget.setValue(int(value))
        widget.blockSignals(False)

    @staticmethod
    def _set_combo_blocked(combo, index):
        combo.blockSignals(True)
        if 0 <= index < combo.count():
            combo.setCurrentIndex(index)
        combo.blockSignals(False)

    def _set_controls_enabled(self, on):
        for w in (self.chk_power, self.bri_slider, self.btn_pick_color, self.fx_combo,
                  self.pal_combo, self.sx_slider, self.ix_slider, self.preset_spin,
                  self.btn_recall, self.btn_save, self.range_start, self.range_stop,
                  self.btn_apply_range, self.ind_edit, self.btn_apply_ind,
                  self.btn_reset_all, self.json_edit, self.btn_send_json, self.btn_query):
            w.setEnabled(on)

    def closeEvent(self, ev):
        if self.driver:
            self.driver.disconnect()
        super().closeEvent(ev)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = WLEDWindow()
    win.resize(640, 860)
    win.show()
    sys.exit(app.exec_())
