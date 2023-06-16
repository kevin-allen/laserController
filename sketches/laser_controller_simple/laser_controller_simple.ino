

const int laserPin = 13; // Digital pin connected to the laser

void setup() {
  pinMode(laserPin, OUTPUT); // Set the laser pin as an output
}

void loop() {
  digitalWrite(laserPin, HIGH); // Turn on the laser
  delay(10); // Wait for 10 ms

  digitalWrite(laserPin, LOW); // Turn off the laser
  delay(100); // Wait for 100 ms 
}



