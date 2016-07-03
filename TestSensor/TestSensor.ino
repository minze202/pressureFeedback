
      
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(9,OUTPUT);
  pinMode(6,OUTPUT);
}

void loop() {
  digitalWrite(9,LOW);
  digitalWrite(6,HIGH);
  delay(1000);
  digitalWrite(6,LOW);
  int sensorValue = analogRead(A4);

  // print the results to the serial monitor:
  Serial.print("sensor = ");
  Serial.println(sensorValue);

  delay(1000);
}
