#include <ESP32Servo.h>
#include <Arduino.h>

// ===================================
// HARDWARE CONFIGURATION
// ===================================
const int numServos = 4;
const int servoPins[numServos] = {13, 14, 22, 23}; 
Servo myServos[numServos];

const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const int NEUTRAL   = 1500;

int currentPulses[numServos] = {NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL};

// ===================================
// CALIBRATION TRIMS (THE FIX)
// ===================================
// Adjust these values to make the arm perfectly vertical at "set"
// Positive values (+) move one way, Negative (-) move the other.
// Start with 0, then try 20, -20, etc.

int TRIMS[numServos] = {
  0,    // J1 Trim (Turntable)
  -50,    // J2 Trim (Shoulder) - Try changing this if it leans forward/back
  0,    // J3 Trim (Elbow)    - Try changing this if the elbow isn't straight
  0     // J4 Trim (Wrist)
};

// ===================================
// KINEMATICS CONSTANTS
// ===================================
const float H_BASE = 62.3;
const float J2_OFF[3] = {0.0, 16.625, 36.276};
const float J3_OFF[3] = {0.0, 0.0, 120.0};
const float J4_OFF[3] = {11.08, 0.0, 93.85};
const float TCP_OFF[3] = {0.0, 4.9, 45.6};

// Scales
const float SCALE_270 = (1.5 * PI) / 2000.0; 
const float SCALE_180 = (1.0 * PI) / 2000.0; 

// ===================================
// FORWARD KINEMATICS LOGIC
// ===================================

float getAngleRad(int index) {
  // We apply the TRIM here. 
  // The math thinks "0 radians" is at (NEUTRAL + TRIM).
  int centerPoint = NEUTRAL + TRIMS[index];
  
  int deltaPulse = currentPulses[index] - centerPoint;
  float angle = 0;

  // Scale
  if (index == 3) angle = deltaPulse * SCALE_180;
  else            angle = deltaPulse * SCALE_270;

  // Direction
  if (index == 2) return -angle; // J3 Inverted
  return angle;
}

void updateForwardKinematics() {
  // ... (Same math logic as before) ...
  float q1 = getAngleRad(0); 
  float q2 = getAngleRad(1); 
  float q3 = getAngleRad(2); 
  float q4 = getAngleRad(3); 

  float c1 = cos(q1), s1 = sin(q1);
  float c2 = cos(q2), s2 = sin(q2);
  float c23 = cos(q2 + q3), s23 = sin(q2 + q3);

  float j2_x = (J2_OFF[0] * c1) - (J2_OFF[1] * s1);
  float j2_y = (J2_OFF[0] * s1) + (J2_OFF[1] * c1);
  float j2_z = H_BASE + J2_OFF[2];

  float j3_x = j2_x; 
  float j3_y = j2_y - (J3_OFF[2] * s2 * c1); 
  float j3_z = j2_z + (J3_OFF[2] * c2);

  float j4_x = j3_x + (J4_OFF[0] * c1);
  float j4_y = j3_y + (J4_OFF[0] * s1) - (J4_OFF[2] * s23 * c1);
  float j4_z = j3_z + (J4_OFF[2] * c23);

  float targetX = j4_x - (TCP_OFF[1] * s1) - (TCP_OFF[2] * s23 * s1); 
  float targetY = j4_y + (TCP_OFF[1] * c1) - (TCP_OFF[2] * s23 * c1);
  float targetZ = j4_z + (TCP_OFF[2] * c23);

  Serial.println("\n--- ROBOT TIP POSITION (mm) ---");
  Serial.print("X: "); Serial.print(targetX);
  Serial.print(" | Y: "); Serial.print(targetY);
  Serial.print(" | Z: "); Serial.println(targetZ);
  Serial.println("--------------------------------");
}

// ===================================
// SETUP & LOOP
// ===================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  for (int i = 0; i < numServos; i++) {
    myServos[i].setPeriodHertz(50);
    myServos[i].attach(servoPins[i], PULSE_MIN, PULSE_MAX);
    // Move to Calculated Neutral (1500 + Trim)
    currentPulses[i] = NEUTRAL + TRIMS[i];
    myServos[i].writeMicroseconds(currentPulses[i]);
  }
  
  Serial.println("System Ready. Adjust TRIMS in code to calibrate.");
  updateForwardKinematics(); 
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input == "set") {
      for (int i = 0; i < numServos; i++) {
        // "Set" now goes to the TRIMMED neutral
        currentPulses[i] = NEUTRAL + TRIMS[i];
        myServos[i].writeMicroseconds(currentPulses[i]);
      }
      updateForwardKinematics();
    } 
    else if (input.startsWith("s")) {
      parseServoCommand(input);
    }
  }
}

void parseServoCommand(String cmd) {
  int dashIndex = cmd.indexOf('-');
  if (dashIndex != -1) {
    int sNum = cmd.substring(1, dashIndex).toInt();
    int pulse = cmd.substring(dashIndex + 1).toInt();

    if (sNum >= 1 && sNum <= numServos && pulse >= PULSE_MIN && pulse <= PULSE_MAX) {
      currentPulses[sNum - 1] = pulse;
      myServos[sNum - 1].writeMicroseconds(pulse);
      updateForwardKinematics();
    }
  }
}