int val = 0;

int throttle = 3;
int rudder = 9;
int aileron = 10;
int elevator = 11;

int throttleVal = 0;
int rudderVal = 127;
int aileronVal = 127;
int elevatorVal = 127;

byte offset[4];

void setup() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  pinMode(throttle, OUTPUT);
  pinMode(rudder, OUTPUT);
  pinMode(aileron, OUTPUT);
  pinMode(elevator, OUTPUT);

  Serial.begin(9600);
  Serial.setTimeout(20);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(offset, 4);

    throttleVal = offset[0];
    rudderVal = offset[1];
    aileronVal = offset[2];
    elevatorVal = offset[3];
  }

  /*

  if (offset[0] < 128) {
    aileronVal = 110;
  } else {
    aileronVal = 144;
  }

  if (offset[1] > 128) {
    elevatorVal = 110;
  } else {
    elevatorVal = 144;
  }

  */
  
  analogWrite(throttle, throttleVal);
  analogWrite(rudder, rudderVal);
  analogWrite(aileron, aileronVal);
  analogWrite(elevator, elevatorVal);
}
