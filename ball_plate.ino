
#include <Servo.h>
#include <math.h>

// --- HARDWARE SETUP ---
Servo motor[3];
const int servoPins[3] = {3, 5, 6}; // Front, Rear Right, Rear Left

// --- PID VARIABLES ---
float ballX, ballY;
float centerX = 320, centerY = 240; // Center of 640x480 camera
float errorX, errorY, lastErrorX, lastErrorY;

float integralX = 0;
float integralY = 0;

// --- PID TUNING ---
// If it overshoots the center, decrease Kp or increase Kd.
// If it stops before reaching the center, increase Ki.
float kp = 0.22;
float ki = 0.12;
float kd = 0.15;

// --- DEADZONE (pixels) ---
// Ball is considered "centered" within this radius. Prevents endless micro-corrections.
const float DEADZONE = 5.0;

// --- TIMING ---
unsigned long lastTime = 0;

// --- STRING BUFFER for serial parsing ---
String inputBuffer = "";

void setup() {
  Serial.begin(115200); // Must match Python script

  for (int i = 0; i < 3; i++) {
    motor[i].attach(servoPins[i]);
    motor[i].write(90); // Initialize at level
  }

  lastTime = micros();
  delay(1000);
}

void loop() {
  // Read all available serial bytes into buffer
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // Full line received — parse it
      processLine(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void processLine(String line) {
  // Expect format: "X Y\n"  or  "-1 -1\n"
  int spaceIdx = line.indexOf(' ');
  if (spaceIdx == -1) return; // Malformed, ignore

  ballX = line.substring(0, spaceIdx).toFloat();
  ballY = line.substring(spaceIdx + 1).toFloat();

  // --- Compute dt in seconds ---
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  if (dt <= 0 || dt > 0.5) dt = 0.033; // Clamp to sane range (~30Hz default)

  // --- Ball Detection Logic ---
  if (ballX == -1) {
    // Ball is off the plate: reset memory and level the platform
    integralX   = 0;
    integralY   = 0;
    lastErrorX  = 0;
    lastErrorY  = 0;
    movePlate(0, 0);
    return;
  }

  // --- Calculate Error ---
  errorX = centerX - ballX;
  errorY = centerY - ballY;

  // --- Deadzone: if close enough, hold level ---

  // --- Integral (with dt) ---
  integralX += errorX * dt;
  integralY += errorY * dt;

  // Anti-windup: prevent runaway tilt if ball is stuck
  integralX = constrain(integralX, -100, 100);
  integralY = constrain(integralY, -100, 100);

  // --- PID Calculation (with dt) ---
  float roll  = (errorX * kp)
              + (integralX * ki)
              + ((errorX - lastErrorX) / dt * kd);

  float pitch = (errorY * kp)
              + (integralY * ki)
              + ((errorY - lastErrorY) / dt * kd);

  // --- Execute Movement ---
  movePlate(pitch, roll);

  // Store current error for next derivative calculation
  lastErrorX = errorX;
  lastErrorY = errorY;
}

void movePlate(float pitch, float roll) {
  /* 120-degree Kinematic Mixing:
     m1 (D3) = Front
     m2 (D5) = Rear Right
     m3 (D6) = Rear Left
  */
  float m1 = 90 + pitch;
  float m2 = 90 - (0.5 * pitch) + (0.866 * roll);
  float m3 = 90 - (0.5 * pitch) - (0.866 * roll);

  // Write to servos with safety constraints (adjust 65/115 for your linkage)
  motor[0].write(constrain(m1, 65, 115));
  motor[1].write(constrain(m2, 65, 115));
  motor[2].write(constrain(m3, 65, 115));
}
