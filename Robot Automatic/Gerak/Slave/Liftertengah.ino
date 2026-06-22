#include <ModbusRtu.h>

#define SLAVE_ID 11
#define EN_PIN   4

#define ENC_A 2
#define ENC_B 3

#define MOTOR_P 5
#define MOTOR_N 6

#define tikup 1800
#define tikdowb 1000 //posisi terakhir naik di hitung

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[3];

volatile long encoderTicks = 0;

enum LiftState
{
  IDLE,
  MOVING_UP,
  MOVING_DOWN,
  FINISHED
};

LiftState liftState = IDLE;
void encoderA()
{
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if(A == B)
    encoderTicks++;
  else
    encoderTicks--;
}

void encoderB()
{
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if(A != B)
    encoderTicks++;
  else
    encoderTicks--;
}
void berhenti()
{
  analogWrite(MOTOR_P,0);
  analogWrite(MOTOR_N,0);
}

void moveUp()
{
  analogWrite(MOTOR_P,80);
  analogWrite(MOTOR_N,0);
}

void moveDown()
{
  analogWrite(MOTOR_P,0);
  analogWrite(MOTOR_N,80);
}
void setup()
{
  Serial.begin(115200);

  pinMode(ENC_A,INPUT_PULLUP);
  pinMode(ENC_B,INPUT_PULLUP);

  pinMode(MOTOR_P,OUTPUT);
  pinMode(MOTOR_N,OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderA,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(ENC_B),
    encoderB,
    CHANGE);

  holdingRegs[0] = 0;
  holdingRegs[1] = 0;
  holdingRegs[2] = 0;

  slave.start();

  // Serial.println("idup");
}

void loop()
{
  slave.poll(holdingRegs,3);

  holdingRegs[1] = (int16_t)encoderTicks;

  switch(liftState)
  {
    case IDLE:
    {
      holdingRegs[2] = 0;

      if(holdingRegs[0] == 1)
      {
        encoderTicks = 0;

        liftState = MOVING_UP;

        // Serial.println("up");
      }

      else if(holdingRegs[0] == 2)
      {
        encoderTicks = 0;

        liftState = MOVING_DOWN;

        // Serial.println("down");
      }
    }
    break;

    case MOVING_UP:
    {
      holdingRegs[2] = 1;

      moveUp();

      if(encoderTicks <= -tikup)
      {
        berhenti();

        holdingRegs[2] = 2;

        liftState = FINISHED;

        Serial.println("UP DONE");
      }
    }
    break;

    case MOVING_DOWN:
    {
      holdingRegs[2] = 1;

      moveDown();

      if(encoderTicks >= tikdowb)
      {
        berhenti();

        holdingRegs[2] = 2;

        liftState = FINISHED;

        Serial.println("DOWN DONE");
      }
    }
    break;

    case FINISHED:
    {
      berhenti();

      holdingRegs[0] = 0;

      // liftState = IDLE;
    }
    break;
  }

  
  // static unsigned long t=0;

  // if(millis()-t > 500)
  // {
  //   t = millis();

  //   Serial.print("CMD=");
  //   Serial.print(holdingRegs[0]);

  //   Serial.print(" TICK=");
  //   Serial.print(encoderTicks);

  //   Serial.print(" STATUS=");
  //   Serial.print(holdingRegs[2]);

  //   Serial.print(" STATE=");
  //   Serial.println(liftState);
  // }
}
