
      
void setup() {
    Serial.begin(9600);  // initialize serial communication at 9600 bits per second:
}

void loop() {   // the loop routine runs over and over again forever:

   long int sensorValue = analogRead(A5); // read the input on analog pin 0:
   long int tensione, P, Pmbar;
    // print out the value you read:
   //'tensione= sensorValue*0.0049;'
   //'P=(sensorValue*0.0049/3.3+0.095)/0.009;'
   //Pmbar=((analogRead(A5)*(0.0048828125)/4.8)+0.095)/0.0009;
   
   //Serial.println(tensione);
   //Serial.println(P);
   //Serial.println(Pmbar);
   Serial.println(sensorValue);
   delay(500); // delay in between reads for stability
}
