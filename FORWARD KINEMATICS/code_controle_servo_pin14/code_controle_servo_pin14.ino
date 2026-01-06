#include <ESP32Servo.h>

// ===================================
// HARDWARE CONFIGURATION
// ===================================
const int SERVO_PIN = 14; 
Servo calibrationServo;

// ===================================
// CONSTANTS
// ===================================
const int PULSE_MIN = 500;
const int PULSE_MAX = 2500;
const int NEUTRAL   = 1500;

void setup() {
  Serial.begin(115200);
  delay(1000);

  calibrationServo.setPeriodHertz(50);
  calibrationServo.attach(SERVO_PIN, PULSE_MIN, PULSE_MAX);

  Serial.println("--- MULTI-POSITION SERVO CALIBRATOR ---");
  Serial.println("Commands:");
  Serial.println("  'set'      -> Center (1500us)");
  Serial.println("  'min'      -> Minimum (500us)");
  Serial.println("  'max'      -> Maximum (2500us)");
  Serial.println("  [number]   -> Any value between 500-2500");
  Serial.println("---------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase(); // Make it case-insensitive

    if (input == "set") {
      moveServo(NEUTRAL, "NEUTRAL");
    } 
    else if (input == "min") {
      moveServo(PULSE_MIN, "MINIMUM");
    } 
    else if (input == "max") {
      moveServo(PULSE_MAX, "MAXIMUM");
    } 
    else if (input.length() > 0) {
      // Check if the input is a number
      int value = input.toInt();
      
      // toInt() returns 0 if it's not a number
      if (value >= PULSE_MIN && value <= PULSE_MAX) {
        moveServo(value, "CUSTOM");
      } else {
        Serial.print("⚠️ Invalid! Enter a number between ");
        Serial.print(PULSE_MIN);
        Serial.print(" and ");
        Serial.println(PULSE_MAX);
      }
    }
  }
}

// Helper function to move and print
void moveServo(int microSeconds, String label) {
  Serial.print(">> Moving to ");
  Serial.print(label);
  Serial.print(": ");
  Serial.print(microSeconds);
  Serial.println("µs");
  
  calibrationServo.writeMicroseconds(microSeconds);
}