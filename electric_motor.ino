/*
 
 */

void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT); //motor controller 5v out
}

void loop() {
  
  int val = analogRead(8); //pedal pot.
  val = map(val, 75, 623, 0, 255);
  analogWrite(7, val);
  
  Serial.println(val);
}
