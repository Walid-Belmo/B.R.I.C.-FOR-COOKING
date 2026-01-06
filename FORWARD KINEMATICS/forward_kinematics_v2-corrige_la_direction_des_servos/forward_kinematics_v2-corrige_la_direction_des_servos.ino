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
// KINEMATICS CONSTANTS (Your Measurements)
// ===================================
const float H_BASE = 62.3;
const float J2_OFF[3] = {0.0, 16.625, 36.276};
const float J3_OFF[3] = {0.0, 0.0, 120.0};
const float J4_OFF[3] = {11.08, 0.0, 93.85};
const float TCP_OFF[3] = {0.0, 4.9, 45.6};

// ===================================
// FORWARD KINEMATICS LOGIC
// ===================================

float getAngleRad(int index) {
  float rawRad = (currentPulses[index] - NEUTRAL) * (PI / 1000.0);
  
  // DIRECTION FIXES
  if (index == 0) return rawRad;       // J1: Positive
  if (index == 1) return rawRad;       // J2: Positive
  
  // FIXED J3: User test confirmed s3-1900 was opposite to thumb. 
  // Inverting to match Right-Hand Rule.
  if (index == 2) return -rawRad;      
  
  if (index == 3) return rawRad;       // J4: Positive
  return 0;
}

void updateForwardKinematics() {
  float q1 = getAngleRad(0); // Turntable (Z-axis)
  float q2 = getAngleRad(1); // Shoulder (X-axis)
  float q3 = getAngleRad(2); // Elbow (X-axis)
  float q4 = getAngleRad(3); // Wrist (Z-axis)

  // Trig Caching
  float c1 = cos(q1), s1 = sin(q1);
  float c2 = cos(q2), s2 = sin(q2);
  float c23 = cos(q2 + q3), s23 = sin(q2 + q3);

  // --- 1. Position of J2 (Turntable Rotation) ---
  float j2_x = (J2_OFF[0] * c1) - (J2_OFF[1] * s1);
  float j2_y = (J2_OFF[0] * s1) + (J2_OFF[1] * c1);
  float j2_z = H_BASE + J2_OFF[2];

  // --- 2. Position of J3 (Shoulder Rotation) ---
  // Shoulder rotates around X. J3 offset [0, 0, 120]
  float j3_x = j2_x; 
  float j3_y = j2_y - (J3_OFF[2] * s2 * c1); 
  float j3_z = j2_z + (J3_OFF[2] * c2);

  // --- 3. Position of J4 (Elbow Rotation) ---
  // Elbow rotates around X. Jump to J4: [11.08, 0, 93.85]
  // The 11.08 is forward (X), 93.85 is up (Z)
  float j4_x = j3_x + (J4_OFF[0] * c1);
  float j4_y = j3_y + (J4_OFF[0] * s1) - (J4_OFF[2] * s23 * c1);
  float j4_z = j3_z + (J4_OFF[2] * c23);

  // --- 4. Final TCP (Tip) Position ---
  // Jump from J4 to Tip: [0, 4.9, 45.6]
  // Simplified for X-plane motion:
  float targetX = j4_x - (TCP_OFF[1] * s1) - (TCP_OFF[2] * s23 * s1); 
  float targetY = j4_y + (TCP_OFF[1] * c1) - (TCP_OFF[2] * s23 * c1);
  float targetZ = j4_z + (TCP_OFF[2] * c23);

  // Serial Output
  Serial.println("\n--- ROBOT TIP POSITION (mm) ---");
  Serial.print("X: "); Serial.print(targetX);
  Serial.print(" | Y: "); Serial.print(targetY);
  Serial.print(" | Z: "); Serial.println(targetZ);
  Serial.println("--------------------------------");
}

// ===================================
// ARDUINO CORE FUNCTIONS
// ===================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  for (int i = 0; i < numServos; i++) {
    myServos[i].setPeriodHertz(50);
    myServos[i].attach(servoPins[i], PULSE_MIN, PULSE_MAX);
    myServos[i].writeMicroseconds(NEUTRAL);
  }
  
  Serial.println("FK System Online. J3 Inversion Applied.");
  updateForwardKinematics(); 
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input == "set") {
      for (int i = 0; i < numServos; i++) {
        currentPulses[i] = NEUTRAL;
        myServos[i].writeMicroseconds(NEUTRAL);
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
      Serial.printf(">> S%d set to %dus\n", sNum, pulse);
      updateForwardKinematics();
    }
  }
}