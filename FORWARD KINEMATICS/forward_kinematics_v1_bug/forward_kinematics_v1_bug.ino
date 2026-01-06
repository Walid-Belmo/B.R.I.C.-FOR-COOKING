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

// Current pulses for each servo
int currentPulses[numServos] = {NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL};

// ===================================
// KINEMATICS STRUCTS & CONSTANTS
// ===================================
struct Point3D {
  float x, y, z;
};

// Based on your measurements:
const float H_BASE = 62.3;
const float J2_OFF[3] = {0.0, 16.625, 36.276};
const float J3_OFF[3] = {0.0, 0.0, 120.0};
const float J4_OFF[3] = {11.08, 0.0, 93.85};
const float TCP_OFF[3] = {0.0, 4.9, 45.6};

// ===================================
// FORWARD KINEMATICS CALCULATOR
// ===================================

// Helper: Converts servo pulse (500-2500) to Radians relative to Neutral (1500)
// Assuming 180 degrees range (1000us span per 90 degrees)
float pulseToRad(int pulse) {
  return (pulse - NEUTRAL) * (PI / 1000.0);
}

void updateForwardKinematics() {
  // Convert current pulses to radians
  float q1 = pulseToRad(currentPulses[0]); // Z rotation
  float q2 = pulseToRad(currentPulses[1]); // X rotation
  float q3 = pulseToRad(currentPulses[2]); // X rotation
  float q4 = pulseToRad(currentPulses[3]); // Z rotation

  // --- Start Chain at Base ---
  // Frame 0: Floor
  float x = 0, y = 0, z = H_BASE;

  // --- Joint 1 to Joint 2 ---
  // J1 rotates around Z. 
  // Jump to J2: [0, 16.625, 36.276]
  float x2 = (J2_OFF[0] * cos(q1)) - (J2_OFF[1] * sin(q1));
  float y2 = (J2_OFF[0] * sin(q1)) + (J2_OFF[1] * cos(q1));
  float z2 = z + J2_OFF[2];

  // --- Joint 2 to Joint 3 ---
  // J2 rotates around X. Jump: [0, 0, 120]
  // We apply rotation of J2 and the jump to J3
  // Simplification for brevity: nested trig chain
  float cos1 = cos(q1), sin1 = sin(q1);
  float cos2 = cos(q2), sin2 = sin(q2);
  float cos3 = cos(q2 + q3), sin3 = sin(q2 + q3); // J3 is parallel to J2

  // --- Final TCP Position (Simplified Matrix multiplication logic) ---
  // This calculates the Tip (x,y,z) relative to the floor
  float targetX = cos1 * (J3_OFF[0] + J4_OFF[0] * cos3 + TCP_OFF[0] * cos3 - TCP_OFF[2] * sin3) 
                - sin1 * (J2_OFF[1] + J4_OFF[2] * sin2 + TCP_OFF[2] * cos3 * sin2);
                
  float targetY = sin1 * (J3_OFF[0] + J4_OFF[0] * cos3 + TCP_OFF[0] * cos3 - TCP_OFF[2] * sin3) 
                + cos1 * (J2_OFF[1] + J4_OFF[2] * sin2 + TCP_OFF[2] * cos3 * sin2);

  float targetZ = H_BASE + J2_OFF[2] + J3_OFF[2] * cos2 + J4_OFF[2] * cos2 + TCP_OFF[2] * cos3;

  Serial.println("\n--- LIVE COORDINATES (mm) ---");
  Serial.print("X: "); Serial.println(targetX);
  Serial.print("Y: "); Serial.println(targetY);
  Serial.print("Z: "); Serial.println(targetZ);
  Serial.println("-----------------------------");
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
    myServos[i].writeMicroseconds(NEUTRAL);
  }
  
  Serial.println("System Ready. Commands: 'set' or 's[num]-[pulse]'");
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
      Serial.printf(">> Servo %d -> %dµs\n", sNum, pulse);
      
      // Every time a servo moves, recalculate where the tip is
      updateForwardKinematics();
    }
  }
}