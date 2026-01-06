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
// CALIBRATION TRIMS (USER DEFINED)
// ===================================
int TRIMS[numServos] = {
  0,    // J1
  -50,  // J2 (Shoulder) - Confirmed -50 centers it
  0,    // J3
  0     // J4
};

// ===================================
// KINEMATICS CONSTANTS (Y-FORWARD FRAME)
// ===================================
const float H_BASE = 62.3;

// J2 (Shoulder)
const float SIDE_SHIFT_J2 = 16.625; // X-axis (Side)
const float HEIGHT_J2     = 36.276; // Z-axis

// J3 (Elbow)
const float LINK_J2_J3 = 120.0;

// J4 (Wrist)
const float FORWARD_SHIFT_J4 = 11.08; // Y-axis (Reach)
const float LINK_J3_J4       = 93.85; // Z-axis (Length)

// Tool
const float SIDE_SHIFT_TOOL = -4.9;   // X-axis (Side) - Corrected Direction
const float LENGTH_TOOL     = 45.6;

// ===================================
// SCALES (CALIBRATED FROM MEASUREMENTS)
// ===================================
// TD-8120MG Calibrated: 1450us -> 2070us = 90 degrees
// Range = 620us per 90 degrees.
const float SCALE_TD8120 = (PI / 2.0) / 620.0;  // approx 0.002533 rad/us

// MG90S (Standard 180)
const float SCALE_MG90S  = (PI) / 2000.0;       // approx 0.001571 rad/us

// ===================================
// FORWARD KINEMATICS LOGIC
// ===================================

float getAngleRad(int index) {
  int centerPoint = NEUTRAL + TRIMS[index];
  int deltaPulse = currentPulses[index] - centerPoint;
  float angle = 0;

  // Apply Calibrated Scales
  if (index == 3) angle = deltaPulse * SCALE_MG90S;   // J4
  else            angle = deltaPulse * SCALE_TD8120;  // J1, J2, J3

  if (index == 2) return -angle; // J3 Inverted
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
  
  // --- YZ PLANE CALCULATION (Reaching Plane) ---
  float r_shoulder = 0; 
  float z_shoulder = H_BASE + HEIGHT_J2;

  // 1. Elbow
  float r_elbow = r_shoulder + (LINK_J2_J3 * sin(q2));
  float z_elbow = z_shoulder + (LINK_J2_J3 * cos(q2));

  // 2. Wrist
  // Rotate rigid bracket by pitch (q2+q3)
  float r_wrist = r_elbow + (FORWARD_SHIFT_J4 * c23) + (LINK_J3_J4 * s23);
  float z_wrist = z_elbow - (FORWARD_SHIFT_J4 * s23) + (LINK_J3_J4 * c23);

  // 3. Tool
  // Mix Side(X) and Reach(Y) based on Wrist Roll (q4)
  float tool_side_local  = SIDE_SHIFT_TOOL; 
  float tool_reach_local = 0;
  float tool_len         = LENGTH_TOOL;

  // Roll q4
  float tool_side_rolled  = (tool_side_local * cos(q4)) - (tool_reach_local * sin(q4));
  float tool_reach_rolled = (tool_side_local * sin(q4)) + (tool_reach_local * cos(q4));

  // Pitch (q2+q3) - affects reach & len, side sticks out
  float r_tool = r_wrist + (tool_reach_rolled * c23) + (tool_len * s23);
  float z_tool = z_wrist - (tool_reach_rolled * s23) + (tool_len * c23);
  
  float side_total = SIDE_SHIFT_J2 + tool_side_rolled;

  // --- WORLD PROJECTION (J1 Yaw) ---
  // Y = Forward Reach, X = Side
  float finalY = (r_tool * cos(q1)) - (side_total * sin(q1));
  float finalX = (r_tool * sin(q1)) + (side_total * cos(q1));
  float finalZ = z_tool;

  Serial.println("\n--- ROBOT TIP POSITION (mm) ---");
  Serial.print("X: "); Serial.print(finalX);
  Serial.print(" | Y: "); Serial.print(finalY);
  Serial.print(" | Z: "); Serial.println(finalZ);
  Serial.println("--------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  for (int i = 0; i < numServos; i++) {
    myServos[i].setPeriodHertz(50);
    myServos[i].attach(servoPins[i], PULSE_MIN, PULSE_MAX);
    currentPulses[i] = NEUTRAL + TRIMS[i];
    myServos[i].writeMicroseconds(currentPulses[i]);
  }
  Serial.println("System Ready. Calibrated Scales Active.");
  updateForwardKinematics(); 
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();
    if (input == "set") {
      for (int i = 0; i < numServos; i++) {
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