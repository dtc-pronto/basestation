#!/usr/bin/env python3
"""
UDP Viewer + Camera Switch GUI
------------------------------

This PyQt6 GUI does two things:

1. Starts/stops `ffplay` to view a single UDP stream (e.g., udp://<your_laptop_ip>:8554)
2. Sends a small JSON command over TCP to the robot's `command_listener_node.py`
   (default port 8001) telling it to switch camera *mode* between 'day' and 'night'.

How to use
* Pick the Robot target using the small toggle row.
* Fill in Command Port (8001), Local Receive IP (your laptop's IP), UDP Port, FPS, Resolution.
* Click **Start View + Apply**: launches ffplay and sends JSON with current mode.
* Click **Switch Camera**: toggles 'day'/'night' and sends JSON.
* Click **Stop View**: kills ffplay (robot may keep streaming until changed on robot).
"""
import json
import shlex
import socket
import sys
from dataclasses import dataclass
from typing import Optional, Dict

from PyQt6.QtCore import QProcess
from PyQt6.QtGui import QTextCursor
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QTextEdit, QVBoxLayout,
    QWidget, QButtonGroup, QRadioButton
)

# ──────────────────────────────────────────────────────────────────────────────
# Named robot targets shown as compact toggles (edit to your environment)
# ──────────────────────────────────────────────────────────────────────────────
ROBOT_TARGETS: Dict[str, str] = {
    "Phobos":  "192.168.129.111",
    "Deimos":  "192.168.129.112",
    "Oberon":  "192.168.129.113",
    "Titania": "192.168.129.114",
}

@dataclass
class StreamCmd:
    mode: str          # 'day' or 'night'
    resolution: str    # '480p' | '720p' | '1080p'
    frame_rate: int
    ip_address: str    # your laptop IP to receive UDP
    port: int          # UDP port

    def to_json(self) -> str:
        return json.dumps({
            "mode": self.mode,
            "resolution": self.resolution,
            "frame_rate": self.frame_rate,
            "ip_address": self.ip_address,
            "port": self.port,
        })

class TcpClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port

    def send_json(self, payload: str, timeout: float = 2.0) -> Optional[str]:
        data = payload.encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((self.host, self.port))
            s.sendall(data)
            try:
                resp = s.recv(1024)
                return resp.decode("utf-8", errors="replace") if resp else None
            except socket.timeout:
                return None

class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UDP Viewer + Camera Switch")
        self.resize(900, 500)

        # ── Robot target as a compact toggle row ──
        self.robot_group = QButtonGroup(self)
        robot_row = QHBoxLayout()
        robot_row.setSpacing(12)
        self._robot_idx_to_ip: Dict[int, str] = {}
        for i, (label, ip) in enumerate(ROBOT_TARGETS.items()):
            rb = QRadioButton(label)
            self.robot_group.addButton(rb, i)
            robot_row.addWidget(rb)
            self._robot_idx_to_ip[i] = ip
            if i == 0:
                rb.setChecked(True)
        robot_row.addStretch(1)
        robot_strip = QWidget()
        robot_strip.setLayout(robot_row)

        # ── Free-input settings ──
        self.cmd_port = QLineEdit("8001")
        self.local_ip = QLineEdit("192.168.129.242")
        self.udp_port = QLineEdit("27000")
        self.fps = QLineEdit("10")
        self.resolution = QComboBox(); self.resolution.addItems(["480p", "720p", "1080p"])
        self.mode = QComboBox(); self.mode.addItems(["day", "night"])

        # ffplay args
        self.ffplay_args = QLineEdit("-fflags nobuffer -flags low_delay -framedrop -fast -hide_banner -loglevel warning")

        # Buttons
        self.btn_start = QPushButton("Start View + Apply")
        self.btn_switch = QPushButton("Switch Camera")
        self.btn_stop = QPushButton("Stop View")
        self.btn_start.clicked.connect(self._start_and_apply)
        self.btn_switch.clicked.connect(self._switch_camera)
        self.btn_stop.clicked.connect(self._stop_view)

        # Log
        self.log = QTextEdit(); self.log.setReadOnly(True)

        # ── Settings form (grid) ──
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        row = 0
        grid.addWidget(QLabel("Command Port"), row, 0); grid.addWidget(self.cmd_port, row, 1); row += 1
        grid.addWidget(QLabel("Local Receive IP"), row, 0); grid.addWidget(self.local_ip, row, 1); row += 1
        grid.addWidget(QLabel("UDP Port"), row, 0); grid.addWidget(self.udp_port, row, 1); row += 1
        grid.addWidget(QLabel("FPS"), row, 0); grid.addWidget(self.fps, row, 1); row += 1
        grid.addWidget(QLabel("Resolution"), row, 0); grid.addWidget(self.resolution, row, 1); row += 1
        grid.addWidget(QLabel("Mode"), row, 0); grid.addWidget(self.mode, row, 1); row += 1

        settings_box = QGroupBox("Settings")
        settings_layout = QVBoxLayout()
        settings_layout.setSpacing(8)
        settings_layout.addWidget(robot_strip)
        settings_layout.addLayout(grid)
        settings_box.setLayout(settings_layout)

        # Buttons row
        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_switch)
        btn_row.addWidget(self.btn_stop)
        btn_row.addStretch(1)

        # Root layout (left content + log)
        left = QVBoxLayout()
        left.addWidget(settings_box)
        left.addWidget(QLabel("ffplay args"))
        left.addWidget(self.ffplay_args)
        left.addLayout(btn_row)
        left.addStretch(1)

        right = QVBoxLayout()
        right.addWidget(QLabel("Log"))
        right.addWidget(self.log)

        root = QHBoxLayout()
        root.addLayout(left, 3)
        root.addLayout(right, 4)

        w = QWidget()
        w.setLayout(root)
        self.setCentralWidget(w)

        # ffplay process handle
        self.proc = QProcess()
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._pipe)
        self.proc.finished.connect(lambda *_: self.log.append("[ffplay exited]"))

    # ------------- helpers -------------
    def _selected_robot_ip(self) -> str:
        idx = self.robot_group.checkedId()
        return self._robot_idx_to_ip[idx]

    def _pipe(self):
        data = self.proc.readAllStandardOutput().data().decode(errors="replace")
        self.log.moveCursor(QTextCursor.MoveOperation.End)
        self.log.insertPlainText(data)
        self.log.ensureCursorVisible()

    def _build_cmd(self, url: str) -> list[str]:
        args = shlex.split(self.ffplay_args.text().strip()) if self.ffplay_args.text().strip() else []
        return ["ffplay", *args, url]

    def _send(self, cfg: StreamCmd) -> Optional[str]:
        try:
            host = self._selected_robot_ip()
            port = int(self.cmd_port.text().strip())
            client = TcpClient(host, port)
            payload = cfg.to_json()
            self.log.append(f"Sending to {host}:{port} => {payload}")
            return client.send_json(payload)
        except Exception as e:
            QMessageBox.critical(self, "Send failed", str(e))
            return None

    def _current_cfg(self) -> Optional[StreamCmd]:
        try:
            return StreamCmd(
                mode=self.mode.currentText(),
                resolution=self.resolution.currentText(),
                frame_rate=int(self.fps.text().strip() or "10"),
                ip_address=self.local_ip.text().strip(),
                port=int(self.udp_port.text().strip() or "8554"),
            )
        except ValueError:
            QMessageBox.critical(self, "Invalid input", "FPS/Port must be integers.")
            return None

    # ------------- actions -------------
    def _start_and_apply(self):
        cfg = self._current_cfg()
        if not cfg:
            return
        # start viewer first so it is ready to receive
        self._start_viewer(cfg)
        resp = self._send(cfg)
        if resp:
            self.log.append(f"Robot response: {resp}")

    def _start_viewer(self, cfg: StreamCmd):
        url = f"udp://{cfg.ip_address}:{cfg.port}"
        self._stop_view()
        cmd = self._build_cmd(url)
        self.log.append("$ " + " ".join(shlex.quote(c) for c in cmd))
        self.proc.start(cmd[0], cmd[1:])

    def _switch_camera(self):
        now = self.mode.currentText()
        next_mode = "night" if now == "day" else "day"
        self.mode.setCurrentText(next_mode)
        cfg = self._current_cfg()
        if not cfg:
            return
        resp = self._send(cfg)
        if resp:
            self.log.append(f"Robot response: {resp}")

    def _stop_view(self):
        if self.proc.state() != QProcess.ProcessState.NotRunning:
            self.proc.kill()
            self.proc.waitForFinished(1500)

    def closeEvent(self, e):
        self._stop_view()
        return super().closeEvent(e)

def main():
    app = QApplication(sys.argv)
    w = Main()
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
