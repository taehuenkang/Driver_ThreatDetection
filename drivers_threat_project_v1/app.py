import cv2
import torch
import numpy as np
from flask import Flask, render_template, Response
from datetime import datetime
import pathlib
import os
import sys
import time
import socket

# 경로 처리 (Windows/Linux 호환)
sys.modules["pathlib._local"] = pathlib
if os.name == 'nt':
    pathlib.PosixPath = pathlib.WindowsPath
else:
    pathlib.WindowsPath = pathlib.PosixPath

from ultralytics.utils.plotting import Annotator, colors

app = Flask(__name__)
log_data = []

# 모델 설정
weights = '/home/KTH/work_yolov5/yolov5/custom/custom.pt'  # 사용자 경로에 맞게 설정
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights, force_reload=True).to(device)
model.conf = 0.6
model.iou = 0.45
model.classes = None

# 긴급 감지 대상 클래스
target_class = 'Weapon_Threat'

# 웹캠 초기화
cap = cv2.VideoCapture(0)

# 타이머 초기화
last_check_time = time.time()
classes_last_minute = set()

# 긴급 로그 저장
def save_emergency_log(timestamp, detected_class):
    with open('emergency_log.txt', 'a') as f:
        f.write(f'{timestamp} - EMERGENCY: {detected_class}\n')

# 긴급 신호 전송
def send_emergency_signal(ip='10.10.16.200', port=9999, message='1'):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(5)
            sock.connect((ip, port))
            sock.sendall(message.encode())
            print(f"📡 Sent emergency signal to {ip}:{port}")
    except Exception as e:
        print(f"❌ Failed to send emergency signal: {e}")

# 영상 스트리밍 및 감지 루프
def generate():
    global log_data, last_check_time, classes_last_minute

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)
        annotator = Annotator(frame, line_width=2, example=str(model.names))
        detected_classes = set()

        if results.xyxy and len(results.xyxy[0]) > 0:
            for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
                class_name = model.names[int(cls)]
                label = f'{class_name} {conf:.2f}'
                annotator.box_label(xyxy, label, color=colors(int(cls), True))
                detected_classes.add(class_name)

            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            log_data.append({'time': timestamp, 'objects': list(detected_classes)})

            # Weapon_Threat 감지 시 로그 및 출력
            if target_class in detected_classes:
                print(f'EMERGENCY: {target_class}')
                save_emergency_log(timestamp, target_class)

            # 최근 1분 감지 클래스 누적
            classes_last_minute.update(detected_classes)

        # 1분마다 Weapon_Threat 감지 여부 확인
        if time.time() - last_check_time >= 60:
            if target_class in classes_last_minute:
                print(f"⚠️ 1분 내 '{target_class}' 감지됨")
                send_emergency_signal()
            classes_last_minute.clear()
            last_check_time = time.time()

        # 프레임 출력
        frame = annotator.result()
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Flask 라우터
@app.route('/')
def index():
    latest = log_data[-1] if log_data else {'time': '-', 'objects': []}
    return render_template('index.html', log=latest)

@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/log')
def show_log():
    return render_template('log.html', logs=log_data[-20:][::-1])

# 서버 실행
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)