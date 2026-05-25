#include <ModbusMaster.h>

#define RXD2 16
#define TXD2 17
#define MAX485_RE_DE 4

HardwareSerial RS485(2);

ModbusMaster encoderX;
ModbusMaster encoderY;+

ModbusMaster motorFL;
ModbusMaster motorFR;
ModbusMaster motorBL;
ModbusMaster motorBR;

float wheelDiameter = 0.08; //8cm
float wheelCircumference = 3.14159 * wheelDiameter; //kelilng
float meterPerTick = wheelCircumference / 4096.0; // meter/tick

long xTicks = 0;
long yTicks = 0;

float posX = 0;
float posY = 0;

long startTicksX = 0;
long startTicksY = 0;

int robotState = 1;

bool stateInit = false;

void preTransmission()
{
    digitalWrite(MAX485_RE_DE, HIGH);
}

void postTransmission()
{
    digitalWrite(MAX485_RE_DE, LOW);
}

long readEncoder(ModbusMaster &node)
{
    uint8_t result;

    result = node.readHoldingRegisters(0, 2);

    if(result == node.ku8MBSuccess)
    {
        uint32_t high = node.getResponseBuffer(0);
        uint32_t low  = node.getResponseBuffer(1);

        uint32_t value = (high << 16) | low;

        return (long)value;
    }

    return 0;
}


void sendMotor(ModbusMaster &motor, int direction, int pwm)
{
    motor.writeSingleRegister(0, direction);
    delay(2);
    motor.writeSingleRegister(1, pwm);
}

void stopAll()
{
    sendMotor(motorFL, 0, 0);
    sendMotor(motorFR, 0, 0);
    sendMotor(motorBL, 0, 0);
    sendMotor(motorBR, 0, 0);
}

void mecanumForward(int pwm)
{
    sendMotor(motorFL, 1, pwm);
    sendMotor(motorFR, 1, pwm);
    sendMotor(motorBL, 1, pwm);
    sendMotor(motorBR, 1, pwm);
}

void mecanumStrafeRight(int pwm)
{
    sendMotor(motorFL, 1, pwm);
    sendMotor(motorFR, -1, pwm);
    sendMotor(motorBL, -1, pwm);
    sendMotor(motorBR, 1, pwm);
}

void setup()
{
    pinMode(MAX485_RE_DE, OUTPUT);

    digitalWrite(MAX485_RE_DE, LOW);

    Serial.begin(115200);

    RS485.begin(9600, SERIAL_8N1, RXD2, TXD2);

    encoderX.begin(1, RS485);
    encoderY.begin(2, RS485);

    motorFL.begin(3, RS485);
    motorFR.begin(4, RS485);
    motorBL.begin(5, RS485);
    motorBR.begin(6, RS485);

    encoderX.preTransmission(preTransmission);
    encoderX.postTransmission(postTransmission);

    encoderY.preTransmission(preTransmission);
    encoderY.postTransmission(postTransmission);

    motorFL.preTransmission(preTransmission);
    motorFL.postTransmission(postTransmission);

    motorFR.preTransmission(preTransmission);
    motorFR.postTransmission(postTransmission);

    motorBL.preTransmission(preTransmission);
    motorBL.postTransmission(postTransmission);

    motorBR.preTransmission(preTransmission);
    motorBR.postTransmission(postTransmission);
}


void loop()
{
    //static bool selesai = false;

    xTicks = readEncoder(encoderX);
    yTicks = readEncoder(encoderY);

    switch(robotState)
    {
        case 0:
        {
            stopAll();
            break;
        }

        case 1: //
        {
            if(!stateInit)
            {
                startTicksX = xTicks;
                stateInit = true;

                Serial.println("STATE 1 : MAJU 1 METER");
            }

            posX = (xTicks - startTicksX) * meterPerTick;

            mecanumForward(120);

            Serial.print("X = ");
            Serial.println(posX);

            if(posX >= 1.0)
            {
                stopAll();

                robotState = 2;
                stateInit = false;

                delay(500);
            }

            break;
        }
        case 2:
        {
            if(!stateInit)
            {
                startTicksY = yTicks;
                stateInit = true;

                Serial.println("STATE 2 : STRAFE KANAN 2 METER");
            }

            posY = (yTicks - startTicksY) * meterPerTick;

            mecanumStrafeRight(120);

            Serial.print("Y = ");
            Serial.println(posY);

            if(posY >= 2.0)
            {
                stopAll();

                robotState = 3;
                stateInit = false;

                delay(500);
            }

            break;
        }

        case 3:
        {
            stopAll();

            Serial.println("SELESAI");

            break;
        }
    }

    delay(10);
}