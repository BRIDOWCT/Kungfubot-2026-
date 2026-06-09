#include <ModbusRtu.h>

#define SLAVE_ID 4
#define EN_PIN   4

#define LB_RPWM 3
#define LB_LPWM 5
#define RB_RPWM 11
#define RB_LPWM 6

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[2];

void setMotorLB(int16_t speed)
{
  speed = constrain(speed, -80, 80);

  if(speed > 0)
  {
    analogWrite(LB_RPWM, speed);
    analogWrite(LB_LPWM, 0);
  }
  else if(speed < 0)
  {
    analogWrite(LB_RPWM, 0);
    analogWrite(LB_LPWM, -speed);
  }
  else
  {
    analogWrite(LB_RPWM, 0);
    analogWrite(LB_LPWM, 0);
  }
}

void setMotorRB(int16_t speed)
{
  speed = constrain(speed, -80, 80);

  if(speed > 0)
  {
    analogWrite(RB_RPWM, speed);
    analogWrite(RB_LPWM, 0);
  }
  else if(speed < 0)
  {
    analogWrite(RB_RPWM, 0);
    analogWrite(RB_LPWM, -speed);
  }
  else
  {
    analogWrite(RB_RPWM, 0);
    analogWrite(RB_LPWM, 0);
  }
}

void stopAllMotor()
{
  analogWrite(LB_RPWM, 0);
  analogWrite(LB_LPWM, 0);

  analogWrite(RB_RPWM, 0);
  analogWrite(RB_LPWM, 0);
}

void setup()
{
  pinMode(EN_PIN, OUTPUT);

  pinMode(LB_RPWM, OUTPUT);
  pinMode(LB_LPWM, OUTPUT);

  pinMode(RB_RPWM, OUTPUT);
  pinMode(RB_LPWM, OUTPUT);

  stopAllMotor();

  Serial.begin(115200);

  slave.start();

  // Serial.println("SLAVE REAR ID 4");
}

void loop()
{
  slave.poll(holdingRegs, 2);

  int16_t LB = (int16_t)holdingRegs[0];
  int16_t RB = (int16_t)holdingRegs[1];

  setMotorLB(LB);
  setMotorRB(RB);

  // static unsigned long lastPrint = 0;

  // if(millis() - lastPrint > 500)
  // {
  //   lastPrint = millis();

  //   Serial.print("LB : ");
  //   Serial.print(LB);

  //   Serial.print(" | RB : ");
  //   Serial.println(RB);
  // }
}
