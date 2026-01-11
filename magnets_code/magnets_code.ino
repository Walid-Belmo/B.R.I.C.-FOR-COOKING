// --- PIN DEFINITIONS ---
const int pinMagnetTop = 23;  // Relay 1 (IN1) - Holds gravity
const int pinMagnetSide = 22; // Relay 2 (IN2) - Prevents tipping

void setup() {
  Serial.begin(115200);

  // Configure pins as outputs
  pinMode(pinMagnetTop, OUTPUT);
  pinMode(pinMagnetSide, OUTPUT);

  // Turn BOTH OFF initially (Active LOW logic: HIGH = OFF)
  digitalWrite(pinMagnetTop, HIGH);
  digitalWrite(pinMagnetSide, HIGH);

  Serial.println("--- Dual Magnet System Ready ---");
  Serial.println("Controls:");
  Serial.println("  'a on'  / 'a off'  -> Top Magnet (Gravity)");
  Serial.println("  'b on'  / 'b off'  -> Side Magnet (Tipping)");
  Serial.println("  'on'    -> BOTH ON");
  Serial.println("  'off'   -> BOTH OFF");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Clean up whitespace

    // --- MAGNET A (TOP) ---
    if (command == "a on") {
      digitalWrite(pinMagnetTop, LOW); // ON
      Serial.println("Top Magnet (A): ENGAGED");
    }
    else if (command == "a off") {
      digitalWrite(pinMagnetTop, HIGH); // OFF
      Serial.println("Top Magnet (A): RELEASED");
    }

    // --- MAGNET B (SIDE) ---
    else if (command == "b on") {
      digitalWrite(pinMagnetSide, LOW); // ON
      Serial.println("Side Magnet (B): ENGAGED");
    }
    else if (command == "b off") {
      digitalWrite(pinMagnetSide, HIGH); // OFF
      Serial.println("Side Magnet (B): RELEASED");
    }

    // --- GLOBAL CONTROLS ---
    else if (command == "on") {
      digitalWrite(pinMagnetTop, LOW);
      digitalWrite(pinMagnetSide, LOW);
      Serial.println("ALL MAGNETS: ENGAGED");
    }
    else if (command == "off") {
      digitalWrite(pinMagnetSide, HIGH); // Release side first
      delay(100);                        // Tiny pause for safety
      digitalWrite(pinMagnetTop, HIGH);  // Release top
      Serial.println("ALL MAGNETS: RELEASED");
    }
  }
}