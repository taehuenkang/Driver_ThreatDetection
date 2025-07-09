# 🚗 운전자 위협 감지 기반 자율 정차 시스템

> **실시간 위협 탐지 + 자율 차선 변경 및 정차**  
> Raspberry Pi, YOLOv5, ROS2, OpenCV 기반 차량 내 안전 대응 시스템

---

## 📌 프로젝트 개요

차량 내부에서 발생할 수 있는 **폭언, 신체 위협, 무기 위협** 등의 위험 상황을 YOLOv5로 실시간 인식하고, **자율주행 TurtleBot**이 **차선을 변경 후 정차**하여 운전자의 안전을 보호하는 시스템입니다. Flask 웹 서버, TCP 통신, ROS2 미들웨어, OpenCV 차선 인식, LCD 시각화 등을 포함한 임베디드 통합 프로젝트입니다.

---

## 🧩 시스템 구성

![system_diagram](./docs/system_diagram.png) <!-- 이미지 경로 수정 필요 -->

| 구성 요소 | 설명 |
|----------|------|
| YOLOv5 객체 탐지 | Normal / Verbal Threat / Physical Threat / Weapon Threat |
| Flask 웹 서버 | YOLO 스트리밍, 위협 감지 시 TCP 이벤트 송신 |
| ROS2 TurtleBot | `/threat`, `/lane_info` 토픽 기반 자율 주행 및 정차 제어 |
| LCD 디스플레이 | 현재 차선 / 위협 상태 출력 |
| OpenCV 기반 차선 인식 | 전처리 및 contour 분석으로 1/2차선 분류 |
| TCP 통신 | Flask ↔ ROS2 간 위협 이벤트 실시간 전달 |

---

## ⚙️ 설치 환경

### 하드웨어
- Raspberry Pi 4 (Ubuntu Server 24.04 LTS)
- USB 카메라 (Webcam)
- TurtleBot3 Burger + OpenCR 보드
- 16x2 LCD 모듈 (I2C)

### 소프트웨어
- Python 3.10, C++
- YOLOv5 (PyTorch 기반)
- Flask
- ROS 2 Jazzy
- OpenCV
- TCP Socket
- RPi.GPIO

---

## 🔧 설치 방법

### 1. Raspberry Pi (YOLO + Flask)
```bash
git clone https://github.com/yourrepo/threat-stop-system.git
cd threat-stop-system/yolo_server

# 가상환경 설치 권장
pip install -r requirements.txt
python app.py  # YOLO 감지 + TCP 송신 + Web 스트리밍
