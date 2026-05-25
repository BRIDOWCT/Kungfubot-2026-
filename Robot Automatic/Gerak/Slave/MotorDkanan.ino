#include <ModbusRtu.h>

#define SLAVE_ID 4  //motor depan kanan
#define TXEN 2

#define RPWM 5
#define LPWM 6

Modbus slave(SLAVE_ID, Serial, TXEN);

uint16_t holdingRegs[10];

void setup()
{
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);

    Serial.begin(9600);

    slave.begin(9600);
}

void setMotor(int dir, int pwm)
{
    pwm = constrain(pwm, 0, 255);

    if(dir == 1)
    {
        analogWrite(RPWM, pwm);
        analogWrite(LPWM, 0);
    }
    else if(dir == -1)
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, pwm);
    }
    else
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
    }
}

void loop()
{
    slave.poll(holdingRegs, 10);

    int direction = (int16_t)holdingRegs[0];
    int pwm = holdingRegs[1];

    setMotor(direction, pwm);
}