from adafruit_servokit import ServoKit
import math
import time
import cv2
import numpy as np
import threading
from flask import Flask, Response, render_template_string, jsonify

# ── Flask App ──────────────────────────────────────────────────────────────────
app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Ballbot Live Feed</title>
    <style>
        body { background: #111; color: #eee; font-family: monospace; display: flex; flex-direction: column; align-items: center; padding: 20px; margin: 0; }
        h1 { color: #0f0; margin-bottom: 10px; }
        img { border: 2px solid #0f0; border-radius: 6px; width: 480px; height: 480px; }
        #stats { margin-top: 14px; font-size: 15px; background: #1a1a1a; border: 1px solid #333; border-radius: 6px; padding: 12px 24px; min-width: 300px; text-align: left; }
        #stats span { color: #0cf; }
        .dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 6px; }
        .green { background: #0f0; }
        .red { background: #f00; }
    </style>
    <script>
        function fetchStats() {
            fetch('/stats').then(r => r.json()).then(d => {
                document.getElementById('imgfps').textContent = d.imgfps.toFixed(1);
                document.getElementById('robfps').textContent = d.robfps.toFixed(1);
                document.getElementById('ballx').textContent = d.ballx;
                document.getElementById('bally').textContent = d.bally;
                document.getElementById('ballarea').textContent = d.ballarea;
                const dot = document.getElementById('balldot');
                dot.className = 'dot ' + (d.ballx !== -1 ? 'green' : 'red');
            });
        }
        setInterval(fetchStats, 200);
    </script>
</head>
<body>
    <h1>Ballbot Live Feed</h1>
    <img src="/video_feed" />
    <div id="stats">
        <div><span id="balldot" class="dot red"></span>
             Ball detected: X=<span id="ballx">--</span>,
             Y=<span id="bally">--</span>,
             Area=<span id="ballarea">--</span></div>
        <br>
        <div> Cam FPS : <span id="imgfps">0.0</span></div>
        <div> Rob FPS : <span id="robfps">0.0</span></div>
    </div>
</body>
</html>
"""


# ── Orient (Robot Kinematics) ──────────────────────────────────────────────────
class Orient:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.kit.servo[8].set_pulse_width_range(500, 2500)
        self.kit.servo[0].set_pulse_width_range(500, 2500)
        self.kit.servo[4].set_pulse_width_range(500, 2500)
        self.s1_index = 8
        self.s2_index = 0
        self.s3_index = 4
        self.L = [0.046, 0.04, 0.065, 0.065]
        self.ini_pos = [0, 0, 0.0732]
        self.pz_max = 0.0732
        self.pz_min = 0.0532
        self.phi_max = 20

    def kinema_inv(self, n, Pz):
        L = self.L
        A = (L[0] + L[1]) / Pz
        B = (Pz**2 + L[2]**2 - (L[0] + L[1])**2 - L[3]**2) / (2 * Pz)
        C = A**2 + 1
        D = 2 * (A * B - (L[0] + L[1]))
        E = B**2 + (L[0] + L[1])**2 - L[2]**2
        Pmx = (-D + math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        Pmz = math.sqrt(L[2]**2 - Pmx**2 + 2 * (L[0] + L[1]) * Pmx - (L[0] + L[1])**2)

        amx = (L[3] / (math.sqrt(n[0]**2 + n[2]**2))) * (n[2])
        amy = 0
        amz = Pz + (L[3] / (math.sqrt(n[0]**2 + n[2]**2))) * (-n[0])
        Am = [amx, amy, amz]
        A = (L[0] - Am[0]) / Am[2]
        B = (Am[0]**2 + Am[1]**2 + Am[2]**2 - L[2]**2 - L[0]**2 + L[1]**2) / (2 * Am[2])
        C = A**2 + 1
        D = 2 * (A * B - L[0])
        E = B**2 + L[0]**2 - L[1]**2
        ax = (-D + math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        ay = 0
        az = math.sqrt(L[1]**2 - ax**2 + 2 * L[0] * ax - L[0]**2)
        if amz < Pmz:
            az = -az
        A2 = [ax, ay, az]
        theta_a = 90 - math.degrees(math.atan2(A2[0] - L[0], A2[2]))

        bmx = (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (-n[2])
        bmy = (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (-math.sqrt(3) * n[2])
        bmz = Pz + (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 + 2 * math.sqrt(3) * n[0] * n[1]))) * (math.sqrt(3) * n[1] + n[0])
        Bm = [bmx, bmy, bmz]
        A = -(Bm[0] + math.sqrt(3) * Bm[1] + 2 * L[0]) / Bm[2]
        B = (Bm[0]**2 + Bm[1]**2 + Bm[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * Bm[2])
        C = A**2 + 4
        D = 2 * A * B + 4 * L[0]
        E = B**2 + L[0]**2 - L[1]**2
        x = (-D - math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        y = math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4 * x**2 - 4 * L[0] * x - L[0]**2)
        if bmz < Pmz:
            z = -z
        B2 = [x, y, z]
        theta_b = 90 - math.degrees(math.atan2(math.sqrt(B2[0]**2 + B2[1]**2) - L[0], B2[2]))

        cmx = (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (-n[2])
        cmy = (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (math.sqrt(3) * n[2])
        cmz = Pz + (L[3] / (math.sqrt(n[0]**2 + 3 * n[1]**2 + 4 * n[2]**2 - 2 * math.sqrt(3) * n[0] * n[1]))) * (-math.sqrt(3) * n[1] + n[0])
        Cm = [cmx, cmy, cmz]
        A = -(Cm[0] - math.sqrt(3) * Cm[1] + 2 * L[0]) / Cm[2]
        B = (Cm[0]**2 + Cm[1]**2 + Cm[2]**2 + L[1]**2 - L[2]**2 - L[0]**2) / (2 * Cm[2])
        C = A**2 + 4
        D = 2 * A * B + 4 * L[0]
        E = B**2 + L[0]**2 - L[1]**2
        x = (-D - math.sqrt(D**2 - 4 * C * E)) / (2 * C)
        y = -math.sqrt(3) * x
        z = math.sqrt(L[1]**2 - 4 * x**2 - 4 * L[0] * x - L[0]**2)
        if cmz < Pmz:
            z = -z
        C2 = [x, y, z]
        theta_c = 90 - math.degrees(math.atan2(math.sqrt(C2[0]**2 + C2[1]**2) - L[0], C2[2]))

        thetas = [theta_a, theta_b - 7, theta_c]
        return thetas

    def control_posture(self, pos, t):
        theta = pos[0]
        phi   = pos[1]
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        n = [x, y, z]
        angles = self.kinema_inv(n, Pz)
        print(f"Servo angles: S1={angles[0]:.2f}° S2={angles[1]:.2f}° S3={angles[2]:.2f}°")
        self.kit.servo[self.s1_index].angle = max(0, min(180, angles[0]))
        self.kit.servo[self.s2_index].angle = max(0, min(180, angles[1]))
        self.kit.servo[self.s3_index].angle = max(0, min(180, angles[2]))
        time.sleep(t)

    def initialize_posture(self):
        self.control_posture(self.ini_pos, 1)

    def safe_shutdown(self):
        try:
            print("[INFO] Moving to home position...")
            self.initialize_posture()
        except Exception as e:
            print(f"[WARN] Could not home servos on shutdown: {e}")

    def cleanup(self):
        pass


# ── PID Controller ─────────────────────────────────────────────────────────────
class PID:
    def __init__(self, KPID, k, alpha):
        self.kp = KPID[0]
        self.ki = KPID[1]
        self.kd = KPID[2]
        self.k = k
        self.alpha = alpha
        self.last_output_x = 0
        self.last_output_y = 0
        self.last_error_x  = 0
        self.integral_x    = 0
        self.last_error_y  = 0
        self.integral_y    = 0
        self.last_time     = None

    def compute(self, goal, current_value):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            return 0, 0
        dt = current_time - self.last_time
        if dt <= 0:
            return 0, 0
        error_x = goal[0] - current_value[0]
        error_y = goal[1] - current_value[1]
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        derivative_x = (error_x - self.last_error_x) / dt
        derivative_y = (error_y - self.last_error_y) / dt
        output_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        output_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        output_x = self.alpha * output_x + (1 - self.alpha) * self.last_output_x
        output_y = self.alpha * output_y + (1 - self.alpha) * self.last_output_y
        theta = math.degrees(math.atan2(output_y, output_x))
        if theta < 0:
            theta += 360
        phi = self.k * math.sqrt(output_x**2 + output_y**2)
        self.last_error_x  = error_x
        self.last_error_y  = error_y
        self.last_output_x = output_x
        self.last_output_y = output_y
        self.last_time     = current_time
        return theta, phi


# ── Camera ─────────────────────────────────────────────────────────────────────
class Camera:
    def __init__(self):
        self.height = self.width = 480
        self.cap = None

        backends = [
            (0, cv2.CAP_V4L2, "index-0 + V4L2"),
            (0, cv2.CAP_ANY,  "index-0 + CAP_ANY"),
        ]

        for dev, backend, label in backends:
            print(f"[CAM] Trying: {label}")
            try:
                cap = cv2.VideoCapture(dev, backend)
            except Exception as e:
                print(f"[CAM] Exception opening {label}: {e}")
                continue
            if not cap.isOpened():
                cap.release()
                print(f"[CAM] Could not open: {label}")
                continue
            ret, _ = cap.read()
            if not ret:
                cap.release()
                print(f"[CAM] Opened but no frame on: {label}")
                continue
            self.cap = cap
            print(f"[CAM] Success with: {label}")
            break

        if self.cap is None:
            raise RuntimeError(
                "\n[ERROR] Cannot open camera. Make sure you ran:\n"
                "  sudo apt install -y python3-opencv\n"
                "and verify V4L2 support with:\n"
                "  python3 -c \"import cv2; "
                "print([l for l in cv2.getBuildInformation().split('\\\\n') if 'V4L' in l])\""
            )

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ── Orange ball HSV range ─────────────────────────────────────────────
        self.lower_orange = np.array([5,  150, 150])
        self.upper_orange = np.array([20, 255, 255])

        self.stop_flag = False

    def take_pic(self):
        self.cap.grab()
        ret, frame = self.cap.retrieve()
        if not ret:
            raise RuntimeError("Failed to grab frame from camera")
        return frame

    def find_ball(self, image):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask   = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.erode(mask,  kernel, iterations=1)
        mask   = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_score   = 0
        min_radius   = 18

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 800:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius < min_radius:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter ** 2)
            if circularity < 0.72:
                continue
            score = circularity * area * radius
            if score > best_score:
                best_score   = score
                best_contour = contour

        if best_contour is not None:
            (cx, cy), radius = cv2.minEnclosingCircle(best_contour)
            area = cv2.contourArea(best_contour)
            x = cx - self.height / 2
            y = cy - self.width  / 2
            x, y = -y, -x
            return int(x), int(y), int(area), (int(cx), int(cy), int(radius))

        return -1, -1, 0, None

    def stop(self):
        self.stop_flag = True

    def cleanup_cam(self):
        self.cap.release()


# ── Globals ────────────────────────────────────────────────────────────────────
# TUNED: Ki drastically reduced (was 0.2909 — caused integral windup → ball drifting to edge)
# TUNED: Kd increased (brakes fast-moving ball before it reaches boundary)
KPID = [0.1091, 0.001, 1.50]
k    = 1.1
a    = 0.60   # slightly less smoothing for faster edge response (was 0.9)

# ── Boundary circle radius in pixels (tune to match your glass disc in frame) ──
BOUNDARY_RADIUS = 140
BOUNDARY_WARN   = 50   # px from edge at which braking kicks in (was 20 — only visual)

latest_jpeg = None
jpeg_lock   = threading.Lock()
stats       = {"imgfps": 0.0, "robfps": 0.0, "ballx": -1, "bally": -1, "ballarea": 0}
stats_lock  = threading.Lock()


# ── Flask routes ───────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            with jpeg_lock:
                frame = latest_jpeg
            if frame is None:
                time.sleep(0.03)
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stats')
def get_stats():
    with stats_lock:
        return jsonify(stats.copy())


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    global latest_jpeg, stats

    robot  = Orient()
    camera = Camera()
    pid    = PID(KPID, k, a)

    robot.initialize_posture()
    pz_ini = robot.ini_pos[2]

    image      = np.zeros((480, 480, 3), dtype=np.uint8)
    image_lock = threading.Lock()
    ball_state = {"x": -1, "y": -1, "area": 0, "circle": None}
    ball_lock  = threading.Lock()
    goal       = [0, 0]
    center_x   = center_y = 240

    # ── Camera capture thread ─────────────────────────────────────────────────
    def get_img():
        nonlocal image
        count = 0
        t0    = time.time()
        while not camera.stop_flag:
            try:
                frame = camera.take_pic()
            except RuntimeError:
                if camera.stop_flag:
                    break
                time.sleep(0.05)
                continue
            with image_lock:
                image = frame
            count += 1
            if count >= 100:
                elapsed = time.time() - t0
                with stats_lock:
                    stats["imgfps"] = round(100 / elapsed, 1) if elapsed > 0 else 0
                t0    = time.time()
                count = 0

    # ── Ball detection thread ─────────────────────────────────────────────────
    def control_rob():
        count = 0
        t0    = time.time()
        while not camera.stop_flag:
            with image_lock:
                frame = image.copy()
            bx, by, barea, bcircle = camera.find_ball(frame)
            with ball_lock:
                ball_state.update({"x": bx, "y": by, "area": barea, "circle": bcircle})
            with stats_lock:
                stats["ballx"]    = bx
                stats["bally"]    = by
                stats["ballarea"] = barea
            count += 1
            if count >= 100:
                elapsed = time.time() - t0
                with stats_lock:
                    stats["robfps"] = round(100 / elapsed, 1) if elapsed > 0 else 0
                t0    = time.time()
                count = 0

    # ── MJPEG encode thread ───────────────────────────────────────────────────
    def encode_feed():
        global latest_jpeg
        while not camera.stop_flag:
            with image_lock:
                frame = image.copy()
            with ball_lock:
                circle = ball_state["circle"]
                bx     = ball_state["x"]
                by     = ball_state["y"]
                barea  = ball_state["area"]
            with stats_lock:
                img_fps = stats["imgfps"]
                rob_fps = stats["robfps"]

            display = frame.copy()

            # ── Boundary circle (glass platform edge) ─────────────────────────
            boundary_color     = (0, 200, 255)   # cyan
            boundary_thickness = 2
            near_edge          = False

            if circle is not None:
                cx_px, cy_px, _ = circle
                dist_px = math.sqrt((cx_px - center_x)**2 + (cy_px - center_y)**2)
                if dist_px > BOUNDARY_RADIUS - BOUNDARY_WARN:
                    boundary_color     = (0, 0, 255)   # red
                    boundary_thickness = 3
                    near_edge          = True

            cv2.circle(display, (center_x, center_y), BOUNDARY_RADIUS,
                       boundary_color, boundary_thickness)

            if near_edge:
                cv2.putText(display, "BOUNDARY!", (center_x - 60, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # ── Center crosshair ──────────────────────────────────────────────
            cv2.line(display,   (center_x - 15, center_y), (center_x + 15, center_y), (0, 255, 0), 1)
            cv2.line(display,   (center_x, center_y - 15), (center_x, center_y + 15), (0, 255, 0), 1)
            cv2.circle(display, (center_x, center_y), 5, (0, 255, 0), 1)

            # ── Ball overlay ──────────────────────────────────────────────────
            if circle is not None:
                cx, cy, radius = circle
                cv2.circle(display, (cx, cy), radius, (0, 165, 255), 2)
                cv2.circle(display, (cx, cy), 4,      (0, 165, 255), -1)
                cv2.line(display,   (cx, cy), (center_x, center_y), (255, 100, 0), 1)
                cv2.putText(display, f"Ball ({bx}, {by})",         (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,  (0, 165, 255), 1)
                cv2.putText(display, f"Area: {barea}  R:{radius}", (cx + 10, cy + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
            else:
                cv2.putText(display, "No ball detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 200), 2)

            cv2.putText(display, f"Cam FPS: {img_fps:.1f}", (10, camera.height - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(display, f"Rob FPS: {rob_fps:.1f}", (10, camera.height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            ret, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                with jpeg_lock:
                    latest_jpeg = jpeg.tobytes()
            time.sleep(0.033)

    # ── Start threads ─────────────────────────────────────────────────────────
    t_cam   = threading.Thread(target=get_img,     name="Thread-Camera",     daemon=True)
    t_rob   = threading.Thread(target=control_rob, name="Thread-BallDetect", daemon=True)
    t_enc   = threading.Thread(target=encode_feed, name="Thread-Encode",     daemon=True)
    t_flask = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False),
        name="Thread-Flask", daemon=True
    )

    t_cam.start()
    t_rob.start()
    t_enc.start()
    t_flask.start()

    import socket
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except Exception:
        ip = "raspberrypi.local"

    print(f"\n[INFO] Ballbot started!")
    print(f"[INFO] Open browser → http://{ip}:5000")
    print(f"[INFO] Press Ctrl+C to stop\n")

    # ── Main control loop ─────────────────────────────────────────────────────
    try:
        while not camera.stop_flag:
            with ball_lock:
                cx = ball_state["x"]
                cy = ball_state["y"]

            if cx != -1:
                theta, phi = pid.compute(goal, [cx, cy])

                # ── Edge braking: boost tilt when ball is near boundary ───────
                # cx, cy here are ball coords relative to center (not pixel coords)
                dist_from_center = math.sqrt(cx**2 + cy**2)
                brake_zone_start = BOUNDARY_RADIUS - BOUNDARY_WARN

                if dist_from_center > brake_zone_start:
                    # How deep into the warning zone (0.0 → 1.0)
                    penetration = (dist_from_center - brake_zone_start) / BOUNDARY_WARN
                    penetration = min(penetration, 1.0)

                    # Scale tilt up to 3.5x at the very edge
                    edge_factor = 1.0 + 2.5 * penetration
                    phi = min(phi * edge_factor, robot.phi_max)

                    # Bleed off integral windup so it doesn't fight the correction
                    pid.integral_x *= 0.5
                    pid.integral_y *= 0.5

                robot.control_posture([theta, phi, pz_ini], 0)

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received. Shutting down...")
    finally:
        camera.stop()
        t_cam.join(timeout=2)
        t_rob.join(timeout=2)
        t_enc.join(timeout=2)
        robot.safe_shutdown()
        robot.cleanup()
        camera.cleanup_cam()
        print("[INFO] Shutdown complete.")


if __name__ == "__main__":
    main()
