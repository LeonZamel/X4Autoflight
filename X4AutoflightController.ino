void setup() {
  // put your setup code here, to run once:
  // pinMode(9, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // analogWrite(9, 255);
  Serial.write("Hi\n");
  delay(1000);
}
