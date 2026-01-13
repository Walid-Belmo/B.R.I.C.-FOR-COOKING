#include <ESP32Servo.h>

// ===================================
// HARDWARE CONFIGURATION
// ===================================

// --- SERVOS ---
const int numServos = 5;
// Pins: {13, 14, 22, 23, 21}
const int servoPins[numServos] = {13, 14, 22, 23, 21}; 
Servo myServos[numServos];

// --- ELECTROMAGNETS (RELAYS) ---
// Note for ARM 2 (No Magnets): These pins will simply do nothing if nothing is connected.
const int PIN_MAGNET_1 = 19; // Top Magnet (Gravity)
const int PIN_MAGNET_2 = 16; // Side Magnet (Tipping)

// Relay Logic (Usually Active LOW)
// LOW = ON (Magnet engaged)
// HIGH = OFF (Magnet released)
const int MAGNET_ON = LOW;
const int MAGNET_OFF = HIGH;

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

  Serial.println("--- DUAL ARM COMPATIBLE FIRMWARE ---");
  Serial.println("(Works for both Arm 1 with magnets and Arm 2 without magnets)");
  
  // 1. Setup Servos
  for (int i = 0; i < numServos; i++) {
    myServos[i].setPeriodHertz(50);
    myServos[i].attach(servoPins[i], PULSE_MIN, PULSE_MAX);
    myServos[i].writeMicroseconds(NEUTRAL); // Initial center
  }

  // 2. Setup Magnets
  pinMode(PIN_MAGNET_1, OUTPUT);
  pinMode(PIN_MAGNET_2, OUTPUT);
  // Default to OFF
  digitalWrite(PIN_MAGNET_1, MAGNET_OFF);
  digitalWrite(PIN_MAGNET_2, MAGNET_OFF);

  Serial.println("System Ready.");
  Serial.println("Servo Cmd: 's1-1500' ... 's5-1500'");
  Serial.println("Magnet Cmd: 'm1-1' (ON), 'm1-0' (OFF)");
}

// ===================================
// MAIN LOOP
// ===================================
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    // COMMAND: SET (Reset servos)
    if (input == "set") {
      Serial.println(">> Centering all servos...");
      for (int i = 0; i < numServos; i++) {
        myServos[i].writeMicroseconds(NEUTRAL);
      }
    } 
    
    // COMMAND: SERVO (s1-1500)
    else if (input.startsWith("s")) {
      parseServoCommand(input);
    }
    
    // COMMAND: MAGNET (m1-1)
    else if (input.startsWith("m")) {
      parseMagnetCommand(input);
    }
  }
}

// ===================================
// HELPER FUNCTIONS
// ===================================

void parseServoCommand(String cmd) {
  // Format: s1-1500
  int dashIndex = cmd.indexOf('-');
  if (dashIndex != -1) {
    int sNum = cmd.substring(1, dashIndex).toInt();
    int pulse = cmd.substring(dashIndex + 1).toInt();

    if (sNum >= 1 && sNum <= numServos) {
      if (pulse >= PULSE_MIN && pulse <= PULSE_MAX) {
        myServos[sNum - 1].writeMicroseconds(pulse);
      }
    }
  }
}

void parseMagnetCommand(String cmd) {
  // Format: m1-1 (Magnet 1 ON), m1-0 (Magnet 1 OFF)
  int dashIndex = cmd.indexOf('-');
  if (dashIndex != -1) {
    int mNum = cmd.substring(1, dashIndex).toInt();
    int state = cmd.substring(dashIndex + 1).toInt(); // 1 or 0

    int pin = -1;
    if (mNum == 1) pin = PIN_MAGNET_1;
    if (mNum == 2) pin = PIN_MAGNET_2;

    if (pin != -1) {
      // Map 1/0 to Relay Logic
      int logicLevel = (state == 1) ? MAGNET_ON : MAGNET_OFF;
      digitalWrite(pin, logicLevel);
    }
  }
}
