#include <Wire.h>
#include <ModbusRtu.h>


#define SLAVE_ID 2 //1 X,2y
#define EN_PIN   4

#define AS5600_ADDR 0x36

//roda bantu (cm)
#define DIAMETER 5.0
#define PI_VALUE 3.14159265

//com
uint16_t holdingRegs[2];

Modbus slave(SLAVE_ID, Serial, EN_PIN);

//var
uint16_t rawAngle = 0;

float angle = 0.0;
float startAngle = 0.0;

int quadrant = 0;
int lastQuadrant = 0;

bool firstRead = true;

long rotationCount = 0;

float totalAngle = 0.0;
float distance = 0.0;

float readAS5600()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE HIGH BYTE
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() >= 2)
  {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();

    rawAngle = ((highByte << 8) | lowByte) & 0x0FFF;

    return rawAngle * 360.0 / 4096.0;
  }

  return angle;
}

void correctAngle()
{
  angle -= startAngle;

  if (angle < 0)
  {
    angle += 360.0;
  }
}

void calculateRotation()
{
  if (angle >= 0 && angle <= 90)
  {
    quadrant = 1;
  }
  else if (angle > 90 && angle <= 180)
  {
    quadrant = 2;
  }
  else if (angle > 180 && angle <= 270)
  {
    quadrant = 3;
  }
  else
  {
    quadrant = 4;
  }

  if (firstRead)
  {
    lastQuadrant = quadrant;
    firstRead = false;
  }

  // CW
  if (quadrant == 1 && lastQuadrant == 4)
  {
    rotationCount++;
  }

  // CCW
  else if (quadrant == 4 && lastQuadrant == 1)
  {
    rotationCount--;
  }

  lastQuadrant = quadrant;

  totalAngle = (rotationCount * 360.0) + angle;
}

void calculateDistance()
{
  distance =
      ((DIAMETER * PI_VALUE) / 360.0) *
      totalAngle;
}

//register 
void updateModbusRegisters()
{
  union
  {
    float value;
    uint16_t reg[2];
  } converter;

  converter.value = distance;

  holdingRegs[0] = converter.reg[0];
  holdingRegs[1] = converter.reg[1];
}

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  slave.start();

  delay(500);

  startAngle = readAS5600();

  Serial.println("AS5600 MODBUS SLAVE READY");
}

void loop()
{
  angle = readAS5600();

  correctAngle();

  calculateRotation();

  calculateDistance();

  updateModbusRegisters();

  slave.poll(holdingRegs, 2);
}
