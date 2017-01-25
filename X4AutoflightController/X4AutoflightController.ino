int val = 0;

void setup() {
  pinMode(9, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    val = Serial.parseInt();
    Serial.println(val, DEC);
    analogWrite(9, val);
  }
}
