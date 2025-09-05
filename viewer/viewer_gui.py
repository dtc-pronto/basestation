#!/usr/bin/env python3
"""
UDP Viewer + Camera Switch GUI (no ROS on laptop)
-------------------------------------------------
This PyQt6 GUI does two things:
  1) Starts/stops `ffplay` to view a single UDP stream (e.g., udp://<your_laptop_ip>:8554)
  2) Sends a small JSON command over TCP to the robot's `command_listener_node.py`
     (default port 8001) telling it to switch camera *mode* between 'day' and 'night'.

Robot side (already provided by you):
- `command_listener_node.py` listens on TCP port 8001 for JSON and republishes it to `/stream_config`. fileciteturn0file2
- `streamer_node.py` subscribes to `/stream_config` and switches topics/ffmpeg based on keys
  {mode, resolution, frame_rate, ip_address, port}. It publishes UDP to `udp://ip_address:port`. fileciteturn0file1

So the laptop GUI only needs: TCP client + ffplay. No ROS needed on the laptop.

How to use (inside container or bare metal)
- Fill in Robot IP (where command_listener_node runs), Command Port (8001),
  Local Receive IP (your laptop's IP), UDP Port (e.g., 8554), FPS, Resolution.
- Click **Start View + Apply**: launches ffplay and sends JSON with current mode.
- Click **Switch Camera**: toggles between 'day' and 'night' and sends JSON again.
- Click **Stop View**: kills ffplay (robot keeps streaming until you switch/stop on robot).

"""
import json
import shlex
import socket
import sys
from dataclasses import dataclass
from typing import Optional

from PyQt6.QtCore import QProcess
from PyQt6.QtGui import QTextCursor
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPushButton, QTextEdit, QVBoxLayout,
    QWidget
)

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
        self.resize(900, 520)

        # --- Controls ---
        self.robot_ip = QLineEdit("192.168.129.112")  # where command_listener_node.py runs
        self.cmd_port = QLineEdit("8001")

        self.local_ip = QLineEdit("192.168.129.242")   # your laptop IP to receive UDP
        self.udp_port = QLineEdit("27000")
        self.fps = QLineEdit("10")

        self.resolution = QComboBox(); self.resolution.addItems(["480p", "720p", "1080p"])
        self.mode = QComboBox(); self.mode.addItems(["day", "night"])  # current/primary

        self.ffplay_args = QLineEdit("-fflags nobuffer -flags low_delay -framedrop -fast -hide_banner -loglevel warning")

        self.btn_start = QPushButton("Start View + Apply")
        self.btn_switch = QPushButton("Switch Camera")
        self.btn_stop = QPushButton("Stop View")

        self.btn_start.clicked.connect(self._start_and_apply)
        self.btn_switch.clicked.connect(self._switch_camera)
        self.btn_stop.clicked.connect(self._stop_view)

        self.log = QTextEdit(); self.log.setReadOnly(True)

        # Layout
        grid = QGridLayout()
        grid.addWidget(QLabel("Robot IP"), 0, 0); grid.addWidget(self.robot_ip, 0, 1)
        grid.addWidget(QLabel("Command Port"), 1, 0); grid.addWidget(self.cmd_port, 1, 1)
        grid.addWidget(QLabel("Local Receive IP"), 2, 0); grid.addWidget(self.local_ip, 2, 1)
        grid.addWidget(QLabel("UDP Port"), 3, 0); grid.addWidget(self.udp_port, 3, 1)
        grid.addWidget(QLabel("FPS"), 4, 0); grid.addWidget(self.fps, 4, 1)
        grid.addWidget(QLabel("Resolution"), 5, 0); grid.addWidget(self.resolution, 5, 1)
        grid.addWidget(QLabel("Mode"), 6, 0); grid.addWidget(self.mode, 6, 1)

        net_box = QGroupBox("Settings"); net_box.setLayout(grid)

        btn_row = QHBoxLayout(); btn_row.addWidget(self.btn_start); btn_row.addWidget(self.btn_switch); btn_row.addWidget(self.btn_stop); btn_row.addStretch(1)

        root = QHBoxLayout()
        left = QVBoxLayout(); left.addWidget(net_box); left.addWidget(QLabel("ffplay args")); left.addWidget(self.ffplay_args); left.addLayout(btn_row); left.addStretch(1)
        right = QVBoxLayout(); right.addWidget(QLabel("Log")); right.addWidget(self.log)
        root.addLayout(left, 3); root.addLayout(right, 4)

        w = QWidget(); w.setLayout(root); self.setCentralWidget(w)

        # ffplay process handle
        self.proc = QProcess()
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._pipe)
        self.proc.finished.connect(lambda *_: self.log.append("[ffplay exited]"))

    # ------------- helpers -------------
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
            host = self.robot_ip.text().strip(); port = int(self.cmd_port.text().strip())
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
        self.log.append("$ " + " ".join(shlex.quote(c) for c in cmd) + "")
        self.proc.start(cmd[0], cmd[1:])

    def _switch_camera(self):
        # toggle mode between day/night and send config (viewer keeps same URL)
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
    w = Main(); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

