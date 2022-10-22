
void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(INPUT,7);
}

// The loop routine runs over and over again forever:
void loop() {
  // Write the sinewave points, followed by the terminator "Carriage Return" and "Linefeed".
  Serial.print("IR ");
  Serial.print(analogRead(A0));
  Serial.write(13);
  Serial.write(10);

  Serial.print("EStop ");
  Serial.print(digitalRead(7));
  Serial.write(13);
  Serial.write(10);

  delay(250);
}
