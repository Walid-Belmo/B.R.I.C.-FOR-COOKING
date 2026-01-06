#include <ESP32Servo.h>

// ===================================
// HARDWARE CONFIGURATION
// ===================================
const int numServos = 4;
const int servoPins[numServos] = {13, 14, 22, 23}; 
Servo myServos[numServos];

// ===================================
// CONSTANTS
// ===================================
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const int NEUTRAL   = 1500;

// ===================================
// SETUP
// ===================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- 4-SERVO INDIVIDUAL CALIBRATOR ---");
  
  for (int i = 0; i < numServos; i++) {
    myServos[i].setPeriodHertz(50);
    myServos[i].attach(servoPins[i], PULSE_MIN, PULSE_MAX);
    myServos[i].writeMicroseconds(NEUTRAL); // Initial center
  }

  Serial.println("Commands:");
  Serial.println("  'set'          -> Center ALL servos (1500us)");
  Serial.println("  's1-1500'      -> Set Servo 1 to 1500us");
  Serial.println("  's4-2200'      -> Set Servo 4 to 2200us");
  Serial.println("  Range allowed: 500 to 2500");
  Serial.println("---------------------------------------");
}

// ===================================
// MAIN LOOP
// ===================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    // COMMAND: SET (All to neutral)
    if (input == "set") {
      Serial.println(">> Centering all servos...");
      for (int i = 0; i < numServos; i++) {
        myServos[i].writeMicroseconds(NEUTRAL);
      }
    } 
    
    // COMMAND: INDIVIDUAL (s1-1500)
    else if (input.startsWith("s")) {
      parseServoCommand(input);
    }
  }
}

// ===================================
// HELPER FUNCTIONS
// ===================================

void parseServoCommand(String cmd) {
  // Format expected: s1-1500
  int dashIndex = cmd.indexOf('-');
  
  if (dashIndex != -1) {
    // Extract servo number (between 's' and '-')
    int sNum = cmd.substring(1, dashIndex).toInt();
    // Extract pulse value (after '-')
    int pulse = cmd.substring(dashIndex + 1).toInt();

    // Validation (Index 1-4 corresponds to array 0-3)
    if (sNum >= 1 && sNum <= numServos) {
      if (pulse >= PULSE_MIN && pulse <= PULSE_MAX) {
        myServos[sNum - 1].writeMicroseconds(pulse);
        
        Serial.print(">> Servo ");
        Serial.print(sNum);
        Serial.print(" moved to ");
        Serial.print(pulse);
        Serial.println("µs");
      } else {
        Serial.println("⚠️ Error: Pulse must be 500-2500");
      }
    } else {
      Serial.println("⚠️ Error: Servo number must be 1, 2, 3, or 4");
    }
  } else {
    Serial.println("⚠️ Error: Format must be s[num]-[pulse] (e.g., s1-1500)");
  }
}