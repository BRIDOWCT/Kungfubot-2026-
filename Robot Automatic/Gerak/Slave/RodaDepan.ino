#include <ModbusRtu.h>

#define SLAVE_ID 3
#define EN_PIN   4


#define LF_RPWM 11
#define LF_LPWM 6
#define RF_RPWM 5
#define RF_LPWM 3

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[2];

void setMotorLF(int16_t speed)
{
  speed = constrain(speed, -80, 80);

  if(speed > 0)
  {
    analogWrite(LF_RPWM, speed);
    analogWrite(LF_LPWM, 0);
  }
  else if(speed < 0)
  {
    analogWrite(LF_RPWM, 0);
    analogWrite(LF_LPWM, -speed);
  }
  else
  {
    analogWrite(LF_RPWM, 0);
    analogWrite(LF_LPWM, 0);
  }
}

void setMotorRF(int16_t speed)
{
  speed = constrain(speed, -80, 80);

  if(speed > 0)
  {
    analogWrite(RF_RPWM, speed);
    analogWrite(RF_LPWM, 0);
  }
  else if(speed < 0)
  {
    analogWrite(RF_RPWM, 0);
    analogWrite(RF_LPWM, -speed);
  }
  else
  {
    analogWrite(RF_RPWM, 0);
    analogWrite(RF_LPWM, 0);
  }
}

void setup()
{
  pinMode(EN_PIN, OUTPUT);

  pinMode(LF_RPWM, OUTPUT);
  pinMode(LF_LPWM, OUTPUT);

  pinMode(RF_RPWM, OUTPUT);
  pinMode(RF_LPWM, OUTPUT);

  analogWrite(LF_RPWM, 0);
  analogWrite(LF_LPWM, 0);

  analogWrite(RF_RPWM, 0);
  analogWrite(RF_LPWM, 0);

  Serial.begin(115200);

  slave.start();

  // Serial.println("SLAVE FRONT ID 3");
}

void loop()
{
  slave.poll(holdingRegs, 2);

  int16_t LF = (int16_t)holdingRegs[0];
  int16_t RF = (int16_t)holdingRegs[1];

  setMotorLF(LF);
  setMotorRF(RF);

  // static unsigned long lastPrint = 0;

  // if(millis() - lastPrint > 500)   //biar ga delay
  // {
  //   lastPrint = millis();

  //   Serial.print("LF : ");
  //   Serial.print(LF);

  //   Serial.print(" | RF : ");
  //   Serial.println(RF);
  // }
}
