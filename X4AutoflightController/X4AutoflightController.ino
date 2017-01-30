int val = 0;

int throttle = 3;
int rudder = 9;
int aileron = 10;
int elevator = 11;

void setup() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  
  pinMode(throttle, OUTPUT);
  pinMode(rudder, OUTPUT);
  pinMode(aileron, OUTPUT);
  pinMode(elevator, OUTPUT);
  
  Serial.begin(115200);
  Serial.setTimeout(20);
}

void loop() {
  if(Serial.available() > 0) {
    val = Serial.parseInt();
  }
  analogWrite(throttle, val);
  analogWrite(rudder, 127);
  analogWrite(aileron, 127);
  analogWrite(elevator, 120);
}
