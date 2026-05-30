import sys
import time
import threading
import struct
import random
import configparser
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QLabel, QFrame)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QImage, QPixmap
import cv2

from protocol import pack_downlink_command, parse_uplink_frame

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mini ROV 地面站 (平滑控制)")
        self.setGeometry(100, 100, 800, 600)


        self.config = configparser.ConfigParser()
        self.config.read('C:/Users/zwm/Desktop/coding_program/the_last_exam_ev/config/config.ini', encoding='utf-8')
        self.overcurrent_threshold = float(self.config.get('overcurrent', 'threshold', fallback='5.0'))
        self.control_freq = self.config.getint('control', 'freq', fallback=20)


        self.thrust_rate = 300.0


        self.keymap = {
            'forward': self.config.get('keyboard', 'forward', fallback='W'),
            'back': self.config.get('keyboard', 'back', fallback='S'),
            'left': self.config.get('keyboard', 'left', fallback='A'),
            'right': self.config.get('keyboard', 'right', fallback='D'),
            'strafe_left': self.config.get('keyboard', 'strafe_left', fallback='Left'),
            'strafe_right': self.config.get('keyboard', 'strafe_right', fallback='Right'),
            'up': self.config.get('keyboard', 'up', fallback='Space'),
            'down': self.config.get('keyboard', 'down', fallback='Shift'),
            'arm_open': self.config.get('keyboard', 'arm_open', fallback='Q'),
            'arm_close': self.config.get('keyboard', 'arm_close', fallback='E'),
            'mode_toggle': self.config.get('keyboard', 'mode_toggle', fallback='X'),
        }


        self.key_states = {
            'forward': False,
            'back': False,
            'left': False,
            'right': False,
            'strafe_left': False,
            'strafe_right': False,
            'up': False,
            'down': False,
        }
        self.arm_angle = 0


        self.current_y = 0.0
        self.current_x = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0


        self.target_y = 0.0
        self.target_x = 0.0
        self.target_z = 0.0
        self.target_yaw = 0.0


        self.current_mode = "SLOW"
        self.mode_index = 0
        self.overcurrent_start_time = None
        self.total_overcurrent_time = 0.0
        self.start_time = time.time()
        self.last_current = 0.0


        self.cap = None
        self.cam_index = 0


        self.sim_running = True

        self.setup_ui()
        self.send_timer = QTimer()
        self.send_timer.timeout.connect(self.send_control)
        self.send_timer.start(1000 // self.control_freq)

        self.update_ui_timer = QTimer()
        self.update_ui_timer.timeout.connect(self.update_ui)
        self.update_ui_timer.start(100)

        self.start_camera()
        self.start_mock_rov()
        self.setFocusPolicy(Qt.StrongFocus)

    def setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        status_layout = QHBoxLayout()
        self.status_label = QLabel("状态: 模拟模式 (平滑控制)")
        self.mode_label = QLabel("模式: 慢速")
        self.current_label = QLabel("电流: 0.00 A")
        self.overcurrent_label = QLabel("过流报警: 无")
        self.total_octime_label = QLabel("累计过流时间: 0.0 s")
        self.uptime_label = QLabel("运行时间: 0 s")
        for w in [self.status_label, self.mode_label, self.current_label,
                  self.overcurrent_label, self.total_octime_label, self.uptime_label]:
            status_layout.addWidget(w)

        self.video_frame = QLabel()
        self.video_frame.setFrameStyle(QFrame.Box)
        self.video_frame.setAlignment(Qt.AlignCenter)
        self.video_frame.setMinimumSize(640, 480)

        key_layout = QHBoxLayout()
        key_layout.addWidget(QLabel(
            f"{self.keymap['forward']}/{self.keymap['back']}:前后  "
            f"{self.keymap['left']}/{self.keymap['right']}:旋转  "
            f"←/→:横移  "
            f"{self.keymap['up']}/{self.keymap['down']}:上下  "
            f"{self.keymap['arm_open']}/{self.keymap['arm_close']}:机械臂  "
            f"{self.keymap['mode_toggle']}:切换模式"
        ))

        layout.addLayout(status_layout)
        layout.addWidget(self.video_frame)
        layout.addLayout(key_layout)

    def start_camera(self):
        self.cap = cv2.VideoCapture(self.cam_index)
        if not self.cap.isOpened():
            self.video_frame.setText("无法打开摄像头")
        else:
            self.camera_timer = QTimer()
            self.camera_timer.timeout.connect(self.update_camera)
            self.camera_timer.start(33)

    def update_camera(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                img = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
                self.video_frame.setPixmap(QPixmap.fromImage(img).scaled(
                    self.video_frame.size(), Qt.KeepAspectRatio))

    def send_control(self):
        dt = 1.0 / self.control_freq
        def approach(current, target, rate, dt):
            diff = target - current
            step = rate * dt
            if abs(diff) <= step:
                return target
            return current + step if diff > 0 else current - step

        self.current_y = approach(self.current_y, self.target_y, self.thrust_rate, dt)
        self.current_x = approach(self.current_x, self.target_x, self.thrust_rate, dt)
        self.current_z = approach(self.current_z, self.target_z, self.thrust_rate, dt)
        self.current_yaw = approach(self.current_yaw, self.target_yaw, self.thrust_rate, dt)

        if self.current_mode == "SLOW":
            scale = 0.3
        elif self.current_mode == "MEDIUM":
            scale = 0.6
        else:
            scale = 1.0

        y_out = self.current_y * scale
        x_out = self.current_x * scale
        z_out = self.current_z * scale
        yaw_out = self.current_yaw * scale

        packet = pack_downlink_command(y_out, x_out, z_out, yaw_out, self.arm_angle)
        if abs(y_out) > 1e-3 or abs(x_out) > 1e-3 or abs(z_out) > 1e-3 or abs(yaw_out) > 1e-3:
            print(f"[推力] Y:{y_out:6.1f}  X:{x_out:6.1f}  Z:{z_out:6.1f}  Yaw:{yaw_out:6.1f}  臂:{self.arm_angle}°")


    def start_mock_rov(self):
        def mock_worker():
            while self.sim_running:
                time.sleep(0.5)
                current = random.uniform(2.0, 8.0)
                temp = 25.0
                leak = 0
                frame_type = 0x53
                data = struct.pack('<fBf', temp, leak, current)
                checksum = frame_type
                for b in data:
                    checksum ^= b
                packet = b'\xFA\xAF' + bytes([frame_type]) + data + bytes([checksum]) + b'\xFB\xBF'
                result = parse_uplink_frame(packet)
                if result:
                    _, _, c = result
                    QTimer.singleShot(0, lambda c=c: self.update_current(c))
        self.sim_running = True
        self.mock_thread = threading.Thread(target=mock_worker, daemon=True)
        self.mock_thread.start()

    def update_current(self, current):
        self.last_current = current
        if current > self.overcurrent_threshold:
            if self.overcurrent_start_time is None:
                self.overcurrent_start_time = time.time()
            self.overcurrent_label.setText("过流报警: 有")
            self.overcurrent_label.setStyleSheet("color: red;")
        else:
            if self.overcurrent_start_time:
                self.total_overcurrent_time += time.time() - self.overcurrent_start_time
                self.overcurrent_start_time = None
            self.overcurrent_label.setText("过流报警: 无")
            self.overcurrent_label.setStyleSheet("color: black;")

    def update_ui(self):
        elapsed = int(time.time() - self.start_time)
        self.uptime_label.setText(f"运行时间: {elapsed} s")
        self.current_label.setText(f"电流: {self.last_current:.2f} A")
        self.total_octime_label.setText(f"累计过流时间: {self.total_overcurrent_time:.1f} s")
        self.mode_label.setText(f"模式: {self.current_mode}")

    def keyPressEvent(self, event):
        key = event.key()
        key_str = self._key_to_str(key, event)  # 传入 event 以便获取文本
        for action, mapped_key in self.keymap.items():
            if mapped_key == key_str:
                if action in self.key_states:
                    self.key_states[action] = True
                    self._update_target_from_states()
                elif action == 'arm_open':
                    self.arm_angle = 45
                    print(f"[机械臂] 张开到 45°")
                elif action == 'arm_close':
                    self.arm_angle = 0
                    print(f"[机械臂] 闭合到 0°")
                elif action == 'mode_toggle':
                    modes = ["SLOW", "MEDIUM", "FAST"]
                    self.mode_index = (self.mode_index + 1) % len(modes)
                    self.current_mode = modes[self.mode_index]
                    print(f"[模式] 切换到 {self.current_mode}")
                break

    def keyReleaseEvent(self, event):
        key = event.key()
        key_str = self._key_to_str(key, event)
        for action, mapped_key in self.keymap.items():
            if mapped_key == key_str and action in self.key_states:
                self.key_states[action] = False
                self._update_target_from_states()
                break

    def _key_to_str(self, key, event):

        if key == Qt.Key_Space:
            return 'Space'
        elif key == Qt.Key_Shift:
            return 'Shift'
        elif key == Qt.Key_Left:
            return 'Left'
        elif key == Qt.Key_Right:
            return 'Right'
        else:
            
            text = event.text()
            if text:
                return text.upper()
            return None

    def _update_target_from_states(self):
        self.target_y = (100.0 if self.key_states['forward'] else 0) - (100.0 if self.key_states['back'] else 0)
        self.target_x = (100.0 if self.key_states['strafe_right'] else 0) - (100.0 if self.key_states['strafe_left'] else 0)
        self.target_z = (100.0 if self.key_states['down'] else 0) - (100.0 if self.key_states['up'] else 0)
        self.target_yaw = (100.0 if self.key_states['right'] else 0) - (100.0 if self.key_states['left'] else 0)

    def closeEvent(self, event):
        self.sim_running = False
        if hasattr(self, 'mock_thread'):
            self.mock_thread.join(timeout=1)
        if self.cap:
            self.cap.release()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())