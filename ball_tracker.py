import cv2
import numpy as np
import serial
import time

# --- SERIAL SETUP ---
PORT = 'COM6'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=0)
    time.sleep(2)
    print(f"Connected to Arduino on {PORT}")
except serial.SerialException as e:
    ser = None
    print(f"WARNING: Arduino not found — {e}")

# --- CAMERA & PLATE CONSTANTS ---
PPCM = 21
RADIUS_PX = int((15 / 2) * PPCM)   # Plate boundary radius in pixels
DEADZONE_PX = 5                    # Center tolerance

# --- THROTTLE ---
SEND_INTERVAL = 0.033              # ~30 Hz
last_send_time = 0.0

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    h, w = frame.shape[:2]
    centerX, centerY = w // 2, h // 2

    # ---------------- RED BALL DETECTION ----------------
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 160, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 160, 100])
    upper_red2 = np.array([180, 255, 255])

    mask = cv2.add(
        cv2.inRange(hsv, lower_red1, upper_red1),
        cv2.inRange(hsv, lower_red2, upper_red2)
    )

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ball_found = False
    ball_x = ball_y = -1
    out_of_bounds = False
    centered = False

    if cnts:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            ball_found = True
            ball_x, ball_y = int(x), int(y)

            dist = ((ball_x - centerX) ** 2 + (ball_y - centerY) ** 2) ** 0.5
            out_of_bounds = dist > RADIUS_PX
            centered = dist <= DEADZONE_PX

            cv2.circle(frame, (ball_x, ball_y), int(radius), (0, 255, 0), 2)

    # ---------------- UI ----------------
    circle_color = (0, 0, 255) if out_of_bounds else (0, 255, 255)
    cv2.circle(frame, (centerX, centerY), RADIUS_PX, circle_color, 2)

    ch_color = (0, 255, 0) if centered else (255, 255, 255)
    cv2.line(frame, (centerX - 15, centerY), (centerX + 15, centerY), ch_color, 1)
    cv2.line(frame, (centerX, centerY - 15), (centerX, centerY + 15), ch_color, 1)

    if centered:
        cv2.circle(frame, (centerX, centerY), 6, (0, 255, 0), -1)
        cv2.putText(frame, "CENTERED", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

    elif out_of_bounds:
        cv2.putText(frame, "OUT OF BOUNDS", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    elif not ball_found:
        cv2.putText(frame, "NO BALL DETECTED", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)

    # ---------------- SEND TO ARDUINO ----------------
    now = time.time()

    if ser and ser.is_open and (now - last_send_time) >= SEND_INTERVAL:
        if ball_found:
            ser.write(f"{ball_x} {ball_y}\n".encode())
        else:
            ser.write(b"-1 -1\n")

        last_send_time = now

    # ---------------- DISPLAY ----------------
    cv2.imshow("3-DOF Ball Plate Control", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------------- CLEANUP ----------------
cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()