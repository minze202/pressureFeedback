
      
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  
  int sensorValue = analogRead(A4);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.println(sensorValue);

  delay(1000);
}
