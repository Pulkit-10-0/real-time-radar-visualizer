# main.py
#!/usr/bin/env python3
# -- coding: utf-8 --

import sys
import time
import cv2
import numpy as np
import serial
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

class TurretTracker(QWidget):
    def __init__(self, port='COM5', baud=9600):
        super().__init__()
        self.setWindowTitle('Laser Turret Tracker')
        self.resize(700, 600)

        # UI setup
        self.image_label = QLabel()
        self.confirm_btn = QPushButton('Confirm Target (C)')
        self.mode_btn = QPushButton('Toggle Auto/Manual (M)')
        self.confirm_btn.clicked.connect(self.confirm_target) 
        self.mode_btn.clicked.connect(self.toggle_mode)
        layout = QVBoxLayout(self)
        layout.addWidget(self.image_label)
        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.mode_btn)

        # Camera initialization
        self.cam = cv2.VideoCapture(1)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cam.isOpened():
            raise IOError('Could not open webcam')

        # Timer for update
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        # Expanded sweep steps for full frame coverage
        self.sweeping = True
        self.sweep_steps = [(x, y) for x in range(0, 181, 20) for y in range(0, 181, 20)]
        self.step_index = 0
        self.last_sweep = time.time()
        self.confirmed = False
        self.auto_mode = True
        self.tracking_window = None
        self.roi_hist = None
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

        # Kalman filter setup
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32)*0.03

        # Servo fine-tuning
        self.offset_x = 0
        self.offset_y = 0

        # Connect to Arduino
        try:
            self.arduino = serial.Serial(port, baud, timeout=1, write_timeout=10)
            time.sleep(2)
            print('✅ Arduino connected')
        except Exception as e:
            self.arduino = None
            print(f'Arduino error: {e}')

    def confirm_target(self):
        if not self.sweeping:
            self.confirmed = True
            print('🎯 Target confirmed')

    def toggle_mode(self):
        if not self.sweeping:
            self.auto_mode = not self.auto_mode
            print(f'🔁 Auto mode: {self.auto_mode}')

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_C:
            self.confirm_target()
        elif e.key() == Qt.Key_M:
            self.toggle_mode()

    def update_frame(self):
        ret, frame = self.cam.read()
        if not ret:
            return
        frame = cv2.flip(frame, 1)
        draw = frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]

        if self.sweeping:
            cv2.putText(draw, '🛠 Calibrating Sweep...', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            if time.time() - self.last_sweep > 0.4:
                x_ang, y_ang = self.sweep_steps[self.step_index]
                if self.arduino:
                    self.arduino.write(f'{x_ang},{y_ang}\n'.encode())
                self.step_index += 1
                self.last_sweep = time.time()
                if self.step_index >= len(self.sweep_steps):
                    self.sweeping = False
                    print('✅ Sweep complete, start tracking')
            rgb = cv2.cvtColor(draw, cv2.COLOR_BGR2RGB)
            qimg = QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888).scaled(640,480,Qt.KeepAspectRatio)
            self.image_label.setPixmap(QPixmap.fromImage(qimg))
            return

        # Detect yellow
        yellow_mask = cv2.inRange(hsv, np.array([15, 100, 100]), np.array([40, 255, 255]))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))

        # Find yellow object
        cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        object_found = False
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > 500:
                x,y,ww,hh = cv2.boundingRect(c)
                cx, cy = x+ww//2, y+hh//2
                object_found = True
                cv2.rectangle(draw, (x,y), (x+ww,y+hh), (0,255,0), 2)
                cv2.circle(draw, (cx,cy), 5, (255,0,0), -1)

                angle_x = int(np.clip(np.interp(cx, [0,w], [0,180]) + self.offset_x, 0, 180))
                angle_y = int(np.clip(np.interp(cy, [0,h], [180,0]) + self.offset_y, 0, 180))

                if self.auto_mode and self.arduino:
                    self.arduino.write(f'{angle_x},{angle_y}\n'.encode())

        # Overlay
        cv2.putText(draw, f'Detected: {object_found}', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0,255,0) if object_found else (0,0,255), 2)

        # Display
        rgb = cv2.cvtColor(draw, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, 3*w, QImage.Format_RGB888).scaled(640,480,Qt.KeepAspectRatio)
        self.image_label.setPixmap(QPixmap.fromImage(qimg))

    def closeEvent(self, ev):
        self.timer.stop()
        self.cam.release()
        if self.arduino:
            self.arduino.close()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    tracker = TurretTracker()
    tracker.show()
    sys.exit(app.exec_())