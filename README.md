#  Self-Balancing Ball Robot

## 📌 Overview
This project is a self-balancing ball robot that uses computer vision and control algorithms to keep a ball centered on a platform. The system uses a Raspberry Pi, camera input, and three servo motors to dynamically adjust the platform in real time.


---
##  📷 Screenshots
<div align="center">
  <img src="https://github.com/user-attachments/assets/099f989c-ea3a-458e-b146-e180f124afed" width="500"/>

  <p><b>Assembled Structure </b></p>
</div>

<br>

<div align="center">
  <img src="https://github.com/user-attachments/assets/5f8cd47d-5307-4310-a238-a16d70e3578a" width="500"/>

  <p><b>Servo Motor</b></p>
</div>

<br>

<div align="center">
  <img src="https://github.com/user-attachments/assets/efb9a0fc-657f-4070-8e32-8ab9595899f4" width="500"/>

  <p><b>Lower Linkage</b></p>
</div>

<br>

<div align="center">
  <img src="https://github.com/user-attachments/assets/f79e521c-b076-4f7d-b021-5de026214127" width="500"/>

  <p><b>Upper Linkage</b></p>
</div>

<br>

<div align="center">
  <img src="https://github.com/user-attachments/assets/f0f74fc0-ce52-4602-b656-73c03971cc9c" width="500"/>

  <p><b>Camera Module</b></p>
</div>
<br><br>

## 🎯 Features
- Real-time ball position tracking using camera  
- Automatic balancing using PID control  
- 3-servo motor platform control  
- Multithreaded processing for better performance  
- Inverse kinematics for precise servo movement  

---

## 🛠️ Tech Stack
- **Language:** Python  
- **Hardware:** Raspberry Pi, Servo Motors, Camera Module  
- **Libraries:**
  - numpy  
  - opencv-python (cv2)  
  - picamera2  
  - adafruit-circuitpython-servokit  
  - math (built-in)  
  - time (built-in)  
  - threading (built-in)  

---
## 🔌 Pin Configuration (Raspberry Pi + Servo Driver)

### 📡 I2C Connection (Raspberry Pi → PCA9685)

| Raspberry Pi Pin | PCA9685 Pin |
|------------------|-------------|
| 3.3V (Pin 1)     | VCC         |
| GND (Pin 6)      | GND         |
| GPIO 2 (SDA)     | SDA         |
| GPIO 3 (SCL)     | SCL         |

---

### ⚙️ Servo Connections

| Servo | Channel |
|-------|---------|
| Servo 1 | CH0 |
| Servo 2 | CH1 |
| Servo 3 | CH2 |

---

### 🔋 Important Power Notes

- Do NOT power servos from Raspberry Pi
- Use external 5V power supply
- Connect all grounds together:
  - Raspberry Pi GND  
  - PCA9685 GND  
  - External power GND  

---
## ⚙️ Installation
```bash
git clone https://github.com/your-username/self-balancing-ball-robot.git
cd self-balancing-ball-robot
chmod +x setup.sh
./setup.sh
