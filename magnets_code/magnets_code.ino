void setup() {
  pinMode(19, OUTPUT);
  pinMode(16, OUTPUT);
}

void loop() {
  digitalWrite(19, LOW);  // Devrait activer le relai 1 (aimant 1)
  digitalWrite(16, HIGH); // Relai 2 désactivé (aimant 2 OFF)
  delay(2000);
  digitalWrite(19, HIGH); // Relai 1 désactivé (aimant 1 OFF)
  digitalWrite(16, LOW);  // Relai 2 activé (aimant 2 ON)
  delay(2000);
}