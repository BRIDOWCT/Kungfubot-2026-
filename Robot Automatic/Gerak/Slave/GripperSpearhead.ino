#include <ModbusRtu.h>
#include <Servo.h>

#define SLAVE_ID 10
#define EN_PIN   4

// Encoder
#define ENC_A 2
#define ENC_B 3

// BTS7960
#define MOTOR_P 5
#define MOTOR_N 6

// Servo
#define SERVO_ARM   9
#define SERVO_GRIP  10

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[3];

/*
holdingRegs[0] = command

0 = idle
1 = scan
2 = grip

holdingRegs[1] = encoder tick

holdingRegs[2] = status

0 = idle
1 = scanning
2 = finished
*/

Servo servoArm;
Servo servoGrip;

volatile long encoderTicks = 0;

enum ArmState
{
  IDLE,
  SCANNING,
  GRIPPING,
  FINISHED
};

ArmState armState = IDLE;

// =====================================================
// ENCODER
// =====================================================

void encoderA()
{
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A == B)
    encoderTicks--;
  else
    encoderTicks++;
}

void encoderB()
{
  bool A = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if (A != B)
    encoderTicks++;
  else
    encoderTicks--;
}

// =====================================================
// MOTOR
// =====================================================

void stopMotor()
{
  analogWrite(MOTOR_P, 0);
  analogWrite(MOTOR_N, 0);
}

// arah dibalik sesuai masalah yang Anda laporkan
void moveScan()
{
  analogWrite(MOTOR_P, 15);
  analogWrite(MOTOR_N, 0);
}

// =====================================================

void setup()
{
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  pinMode(MOTOR_P, OUTPUT);
  pinMode(MOTOR_N, OUTPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderA,
    CHANGE);

  attachInterrupt(
    digitalPinToInterrupt(ENC_B),
    encoderB,
    CHANGE);

  // servoArm.attach(SERVO_ARM);
  servoGrip.attach(SERVO_GRIP);

  // posisi awal

  // servoArm.write(90);
  servoGrip.write(90);

  holdingRegs[0] = 0;
  holdingRegs[1] = 0;
  holdingRegs[2] = 0;

  slave.start();

  Serial.println("SLAVE 10 READY");
}

// =====================================================

void loop()
{
  slave.poll(holdingRegs, 3);

  holdingRegs[1] = (int16_t)encoderTicks;

  switch(armState)
  {
    // ==========================================
    case IDLE:
    {
      holdingRegs[2] = 0;

      if(holdingRegs[0] == 1)
      {
        armState = SCANNING;

        Serial.println("START SCAN");
      }
    }
    break;

    // ==========================================
    case SCANNING:
    {
      holdingRegs[2] = 1;

      moveScan();

      if(holdingRegs[0] == 2)
      {
        stopMotor();

        armState = GRIPPING;

        Serial.println("OBJECT DETECTED");
      }
    }
    break;

    // ==========================================
    case GRIPPING:
    {
      Serial.println("GRIP");

      servoGrip.write(0);

      delay(1000);

      Serial.println("ARM");

      // servoArm.write(0);

      delay(1000);

      holdingRegs[2] = 2;

      armState = FINISHED;
    }
    break;

    // ==========================================
    case FINISHED:
    {
      stopMotor();

      holdingRegs[2] = 2;

      // reset command supaya tidak retrigger
      holdingRegs[0] = 0;
    }
    break;
  }

  static unsigned long t = 0;

  if(millis() - t > 500)
  {
    t = millis();

    Serial.print("CMD=");
    Serial.print(holdingRegs[0]);

    Serial.print(" STATE=");
    Serial.print(armState);

    Serial.print(" STATUS=");
    Serial.print(holdingRegs[2]);

    Serial.print(" TICK=");
    Serial.println(encoderTicks);
  }
}
