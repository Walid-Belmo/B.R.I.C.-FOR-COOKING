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
// KINEMATICS CONSTANTS
// ===================================
const float H_BASE = 62.3;
const float J2_OFF[3] = {0.0, 16.625, 36.276};
const float J3_OFF[3] = {0.0, 0.0, 120.0};
const float J4_OFF[3] = {11.08, 0.0, 93.85};
const float TCP_OFF[3] = {0.0, 4.9, 45.6};

// ===================================
// SERVO SCALING CONSTANTS (NEW!)
// ===================================
// Scale = Total Radians / Pulse Span (2000us)
// 270 degrees = 1.5 * PI
// 180 degrees = 1.0 * PI

const float SCALE_270 = (1.5 * PI) / 2000.0; // For TD-8120MG
const float SCALE_180 = (1.0 * PI) / 2000.0; // For MG90S

// ===================================
// FORWARD KINEMATICS LOGIC
// ===================================

float getAngleRad(int index) {
  int deltaPulse = currentPulses[index] - NEUTRAL;
  float angle = 0;

  // 1. APPLY SCALE based on Servo Type
  if (index == 0) angle = deltaPulse * SCALE_270; // J1: 270 deg
  if (index == 1) angle = deltaPulse * SCALE_270; // J2: 270 deg
  if (index == 2) angle = deltaPulse * SCALE_270; // J3: 270 deg
  if (index == 3) angle = deltaPulse * SCALE_180; // J4: 180 deg (MG90S)

  // 2. APPLY DIRECTION CORRECTIONS (From previous debugging)
  // J1 & J2 were Positive. J3 was Inverted. J4 Positive.
  
  if (index == 2) return -angle; // INVERTED J3
  
  return angle;
}

void updateForwardKinematics() {
  float q1 = getAngleRad(0); 
  float q2 = getAngleRad(1); 
  float q3 = getAngleRad(2); 
  float q4 = getAngleRad(3); 

  float c1 = cos(q1), s1 = sin(q1);
  float c2 = cos(q2), s2 = sin(q2);
  float c23 = cos(q2 + q3), s23 = sin(q2 + q3);

  // Position of J2 center
  float j2_x = (J2_OFF[0] * c1) - (J2_OFF[1] * s1);
  float j2_y = (J2_OFF[0] * s1) + (J2_OFF[1] * c1);
  float j2_z = H_BASE + J2_OFF[2];

  // Position of J3 center
  float j3_x = j2_x; 
  float j3_y = j2_y - (J3_OFF[2] * s2 * c1); 
  float j3_z = j2_z + (J3_OFF[2] * c2);

  // Position of J4 center
  float j4_x = j3_x + (J4_OFF[0] * c1);
  float j4_y = j3_y + (J4_OFF[0] * s1) - (J4_OFF[2] * s23 * c1);
  float j4_z = j3_z + (J4_OFF[2] * c23);

  // Final Tip (TCP)
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
  
  Serial.println("FK System Online. 270deg Scaling Applied.");
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
      Serial.printf(">> Servo %d moved to %dus\n", sNum, pulse);
      updateForwardKinematics();
    }
  }
}