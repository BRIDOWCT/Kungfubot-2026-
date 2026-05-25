#include <Wire.h>
#include <ModbusRtu.h>

#define AS5600_ADDR 0x36



#define SLAVE_ID 1  // as maju mundur x
#define TXEN 2

Modbus slave(SLAVE_ID, Serial, TXEN);

uint16_t holdingRegs[10];

int highByte;
int lowByte;

uint16_t raw_angle;

float angle;
float set_angle;

int quadrant;
int last_quadrant;

bool first_run = true;

long rotation = 0;

float total_angle;

long totalTicks;

void readAS5600()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();

    Wire.requestFrom(AS5600_ADDR, 2);

    if(Wire.available() == 2)
    {
        highByte = Wire.read();
        lowByte = Wire.read();

        raw_angle = (((highByte << 8) | lowByte) & 0x0FFF);

        angle = raw_angle * 360.0 / 4096.0;
    }
}

void correctAngle()
{
    angle = angle - set_angle;

    if(angle < 0)
    {
        angle += 360;
    }
}

void calculateRotation()
{
    if((angle >= 0) && (angle <= 90))
    {
        quadrant = 1;
    }
    else if((angle > 90) && (angle <= 180))
    {
        quadrant = 2;
    }
    else if((angle > 180) && (angle <= 270))
    {
        quadrant = 3;
    }
    else
    {
        quadrant = 4;
    }

    if(first_run)
    {
        last_quadrant = quadrant;
        first_run = false;
    }

    if((quadrant == 1) && (last_quadrant == 4))
    {
        rotation++;
    }
    else if((quadrant == 4) && (last_quadrant == 1))
    {
        rotation--;
    }

    last_quadrant = quadrant;

    total_angle = (rotation * 360.0) + angle;

    totalTicks = total_angle * (4096.0 / 360.0);
}
void setup()
{
    Wire.begin();

    Serial.begin(9600);

    slave.begin(9600);

    readAS5600();

    set_angle = angle;
}

void loop()
{
    readAS5600();

    correctAngle();

    calculateRotation();

    uint32_t temp = (uint32_t)totalTicks;

    holdingRegs[0] = (temp >> 16) & 0xFFFF;
    holdingRegs[1] = temp & 0xFFFF;

    slave.poll(holdingRegs, 10);
}