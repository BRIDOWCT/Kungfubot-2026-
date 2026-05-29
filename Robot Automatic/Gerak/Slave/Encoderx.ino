#include <Wire.h>

#define AS5600 0x36

#define RS485_DIR 4

int Highbyte;
int Lowbyte;

uint16_t raw_angle;

float angle;

float set_angle;

int kuadran;
int last_kuadran;

bool first_condition = true;

int rotation;

float total_angle;

float meter;

float wheelDiameter = 0.05;

float circumference =
  3.14159 * wheelDiameter;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  pinMode(RS485_DIR, OUTPUT);

  digitalWrite(RS485_DIR, LOW);

  Output();

  set_angle = angle;

  Serial.println("SENSOR X READY");
}

void loop() {

  Output();

  CorrectAngle();

  Rotation();

  calculateMeter();

  sendData();

  debugSensor();

  delay(20);
}

void Output(){

  Wire.beginTransmission(AS5600);

  Wire.write(0x0E);

  Wire.endTransmission();

  Wire.requestFrom(AS5600, 2);

  if (Wire.available() == 2){

    Highbyte = Wire.read();

    Lowbyte = Wire.read();

    raw_angle =
      (((Highbyte << 8) | Lowbyte) & 0xFFF);

    angle =
      raw_angle * 360.0 / 4096.0;
  }
}

void CorrectAngle(){

  angle = angle - set_angle;

  if (angle < 0){

    angle = angle + 360;
  }
}

void Rotation(){

  if ((angle >= 0) && (angle <= 90)){
    kuadran = 1;
  }

  else if ((angle > 90) && (angle <= 180)){
    kuadran = 2;
  }

  else if ((angle > 180) && (angle <= 270)){
    kuadran = 3;
  }

  else if ((angle > 270) && (angle <= 360)){
    kuadran = 4;
  }

  if (first_condition) {

    last_kuadran = kuadran;

    first_condition = false;
  }

  if ((kuadran == 1) &&
      (last_kuadran == 4)){

    rotation++;
  }

  else if ((kuadran == 4) &&
           (last_kuadran == 1)){

    rotation--;
  }

  last_kuadran = kuadran;

  total_angle =
    rotation * 360 + angle;
}

void calculateMeter() {

  meter =
    (total_angle / 360.0) *
    circumference;
}

void sendData() {

  digitalWrite(RS485_DIR, HIGH);

  Serial.print("X,");

  Serial.println(meter,3);

  delay(2);

  digitalWrite(RS485_DIR, LOW);
}

void debugSensor() {

  Serial.print("Meter X : ");

  Serial.println(meter);
}
