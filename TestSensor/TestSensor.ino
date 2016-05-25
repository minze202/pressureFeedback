
int sensorValue = 0;        

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(A5, INPUT);
}

void loop() {
  
  sensorValue = analogRead(A5);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.println(sensorValue);

  delay(1000);
}
