#include <ModbusRtu.h>

#define SLAVE_ID 5
#define EN_PIN   4

#define RPWM 6
#define LPWM 5

#define ENC_A 2
#define ENC_B 3

#define POS_BELAKANG  -2000
#define POS_DEPAN      0

const int PWM_SPEED = 30;

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[1];

volatile long encoderTick = 0;

bool motorRunning = false;
int currentCommand = 0;

// ======================================================
// ENCODER
// ======================================================

void encoderA()
{
  if (digitalRead(ENC_A) == digitalRead(ENC_B))
    encoderTick++;
  else
    encoderTick--;
}

void encoderB()
{
  if (digitalRead(ENC_A) != digitalRead(ENC_B))
    encoderTick++;
  else
    encoderTick--;
}

// ======================================================
// MOTOR
// ======================================================

void motorKanan(int pwm)
{
  analogWrite(RPWM, pwm);
  analogWrite(LPWM, 0);
}

void motorKiri(int pwm)
{
  analogWrite(RPWM, 0);
  analogWrite(LPWM, pwm);
}

void motorStop()
{
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

// ======================================================

void setup()
{
  Serial.begin(115200);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderB, CHANGE);

  motorStop();

  encoderTick = 0;

  slave.start();

  Serial.println("Slave Ready");
}

// ======================================================

void loop()
{
  slave.poll(holdingRegs, 1);

  // =====================================
  // Terima command baru
  // =====================================

  if (!motorRunning)
  {
    if (holdingRegs[0] == 1)
    {
      currentCommand = 1;
      motorRunning = true;

      Serial.println("Menuju BELAKANG");
    }
    else if (holdingRegs[0] == 2)
    {
      currentCommand = 2;
      motorRunning = true;

      Serial.println("Menuju DEPAN");
    }
  }

  // =====================================
  // Eksekusi command 1
  // =====================================

  if (motorRunning && currentCommand == 1)
  {
    motorKiri(PWM_SPEED);

    if (encoderTick <= POS_BELAKANG)
    {
      motorStop();

      motorRunning = false;
      currentCommand = 0;
      holdingRegs[0] = 0;

      Serial.println("Posisi BELAKANG tercapai");
    }
  }

  // =====================================
  // Eksekusi command 2
  // =====================================

  if (motorRunning && currentCommand == 2)
  {
    motorKanan(PWM_SPEED);

    if (encoderTick >= POS_DEPAN)
    {
      motorStop();

      motorRunning = false;
      currentCommand = 0;
      holdingRegs[0] = 0;

      Serial.println("Posisi DEPAN tercapai");
    }
  }

  // =====================================
  // Debug
  // =====================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 200)
  {
    lastPrint = millis();

    Serial.print("Tick: ");
    Serial.print(encoderTick);

    Serial.print(" CMD: ");
    Serial.print(holdingRegs[0]);

    Serial.print(" RUN: ");
    Serial.println(motorRunning);
  }
}
