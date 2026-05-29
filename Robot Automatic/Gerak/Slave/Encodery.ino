#include <Wire.h>

#define AS5600 0x36

#define RS485_EN 4

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

String incoming = "";

void setup() {

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  Serial.begin(115200);

  Wire.begin();

  delay(1000);

  ReadAngle();
  set_angle = angle;
}

void loop() {

  ReadAngle();
  CorrectAngle();
  Rotation();

  ReceiveCommand();
}

void ReceiveCommand() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      incoming.trim();
// encoder Y di 2
      if (incoming == "2") {
        SendData();
      }

      incoming = "";
    }
    else {
      incoming += c;
    }
  }
}

void SendData() {

  digitalWrite(RS485_EN, HIGH);

  Serial.print(total_angle);
  Serial.print("\n");

  Serial.flush();

  delay(2);

  digitalWrite(RS485_EN, LOW);
}

void ReadAngle() {

  Wire.beginTransmission(AS5600);
  Wire.write(0x0E);
  Wire.endTransmission();

  Wire.requestFrom(AS5600, 2);

  if (Wire.available() == 2) {

    Highbyte = Wire.read();
    Lowbyte = Wire.read();

    raw_angle = (((Highbyte << 8) | Lowbyte) & 0x0FFF);

    angle = raw_angle * 360.0 / 4096.0;
  }
}

void CorrectAngle() {

  angle = angle - set_angle;

  if (angle < 0) {
    angle += 360;
  }
}

void Rotation() {

  if ((angle >= 0) && (angle <= 90)) {
    kuadran = 1;
  }
  else if ((angle > 90) && (angle <= 180)) {
    kuadran = 2;
  }
  else if ((angle > 180) && (angle <= 270)) {
    kuadran = 3;
  }
  else {
    kuadran = 4;
  }

  if (first_condition) {
    last_kuadran = kuadran;
    first_condition = false;
  }

  if ((kuadran == 1) && (last_kuadran == 4)) {
    rotation++;
  }
  else if ((kuadran == 4) && (last_kuadran == 1)) {
    rotation--;
  }

  last_kuadran = kuadran;

  total_angle = rotation * 360 + angle;
}
