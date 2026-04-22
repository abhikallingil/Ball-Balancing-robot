#  Self-Balancing Ball Robot

## 📌 Overview
This project involves a self-balancing ball robot that uses computer vision and control techniques to keep the ball at the center of a platform. It utilizes a Raspberry Pi, a camera for input, and three servo motors to adjust the platform in real time continuously.


---
##  📷 Screenshots
<table width="100%" border="0">
<tr>
<td width="50%" align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/3d3e28f4-178e-4782-84ad-06a53b513eaa" width="100%" style="border-radius: 8px;"/>



<b>Assembled Structure</b>
</td>
<td width="50%" align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/5f8cd47d-5307-4310-a238-a16d70e3578a" width="100%" style="border-radius: 8px;"/>



<b>Servo Motor</b>
</td>
</tr>
<tr>
<td align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/efb9a0fc-657f-4070-8e32-8ab9595899f4" width="100%" style="border-radius: 8px;"/>



<b>Lower Linkage</b>
</td>
<td align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/f79e521c-b076-4f7d-b021-5de026214127" width="100%" style="border-radius: 8px;"/>



<b>Upper Linkage</b>
</td>
</tr>
<tr>
<td align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/f0f74fc0-ce52-4602-b656-73c03971cc9c" width="100%" style="border-radius: 8px;"/>



<b>Camera Module</b>
</td>
<td align="center" style="padding: 15px; border: 1px solid #d1d1d1; border-radius: 12px; background-color: #fcfcfc;">
<img src="https://github.com/user-attachments/assets/b341c065-658f-4e87-9d06-eb8b12edd8f4" width="100%" style="border-radius: 8px;"/>



<b>Camera View</b>
</td>
</tr>
</table>

---

## ⚙️ How It Works

1. The camera captures real-time video of the platform.
2. Computer vision is used to detect the position of the ball.
3. The system calculates the error between the ball’s current position and the center.
4. A control algorithm (e.g., PID) processes this error.
5. Based on the output, three servo motors adjust the platform’s tilt.
6. This process repeats continuously to keep the ball balanced at the center.

---

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

## 📚 References

- https://www.youtube.com/watch?v=KnYSuQEBGHc&t=171s  
- https://www.youtube.com/watch?v=bi4151fWoTY
---

## ⚙️ Installation

```bash
git clone https://github.com/your-username/self-balancing-ball-robot.git
cd self-balancing-ball-robot
chmod +x setup.sh
./setup.sh
