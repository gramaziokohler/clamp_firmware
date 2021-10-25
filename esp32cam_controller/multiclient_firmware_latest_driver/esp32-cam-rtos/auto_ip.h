IPAddress ip_address() {
  // Test connection to pin 13
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  digitalWrite(13, LOW);
  delay(100);
  boolean c_12_13 = digitalRead(12) == LOW;
  boolean c_13_15 = digitalRead(15) == LOW;
  pinMode(13, INPUT_PULLUP);

  // Test connection to pin 14
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  boolean c_14_15 = digitalRead(15) == LOW;
  pinMode(14, INPUT_PULLUP);

  // Refer to documentation for address table
  if (!c_12_13 && c_14_15) return IPAddress (192, 168, 1, 101);
  if (c_12_13 && !c_14_15) return IPAddress (192, 168, 1, 102);
  if (c_12_13 && c_14_15) return IPAddress (192, 168, 1, 103);
  if (c_13_15) return IPAddress (192, 168, 1, 104);
  return IPAddress (192, 168, 1, 100);
}
