#include <ModbusRtu.h>
#include <Servo.h>

#define SLAVE_ID 10
#define EN_PIN   4

// Encoder
#define ENA 3
#define ENB 2

// BTS7960
#define MOTOR_P 6
#define MOTOR_N 5

// Servo
// #define SERVO_ARM   9
// #define SERVO_GRIP  10

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[2] = {0,0};

int com = 0;

unsigned long t = 0;

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

// Servo servoArm;
// Servo servoGrip;

volatile long encoderTicks = 0;

int error = 0;
int lastError = 0;
int integral = 0;
int derivative = 0;

float kp = 2.0;
float ki = 0.0;
float kd = 0.2;

enum ArmState
{
  IDLE,
  GRIPPING,
  FINISHED
};

ArmState armState = IDLE;

// =====================================================
// ENCODER
// =====================================================

void enc(){
  if(digitalRead(ENB)==HIGH){
    encoderTicks++;
  } else{
    encoderTicks--;
  }
}

// =====================================================
// MOTOR
// =====================================================

void stopMotor()
{
  analogWrite(MOTOR_P, 0);
  analogWrite(MOTOR_N, 0);
}

void PID(int target){

  error = target - encoderTicks;
  integral += error;
  derivative = (error - lastError);

  float output = kp*error + ki*integral + kd*derivative;

  lastError = error;

  output = constrain(output, -30, 30);

  static bool trigger = false;
  if (!trigger){
    t = millis();
    trigger = true;
  } 

  if ((abs(error) > 2) && (millis() - t < 5000)){
    if (output > 0){
      analogWrite(MOTOR_N, output);
      analogWrite(MOTOR_P, 0);
    } 
    else {
      analogWrite(MOTOR_N, 0);
      analogWrite(MOTOR_P, abs(output));
      // Serial.println("TESTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT");
    } 
  } 
  else {
    integral = 0;
    analogWrite(MOTOR_N, 0);
    analogWrite(MOTOR_P, 0);
    // servoGrip.write(0);
    armState = FINISHED;
  }
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

  pinMode(ENA, INPUT_PULLUP);
  pinMode(ENB, INPUT_PULLUP);

  pinMode(MOTOR_P, OUTPUT);
  pinMode(MOTOR_N, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENA), enc, RISING);

  // servoArm.attach(SERVO_ARM);
  // servoGrip.attach(SERVO_GRIP);

  // posisi awal

  // servoArm.write(90);
  // 7servoGrip.write(90);

  holdingRegs[0] = 0;
  holdingRegs[1] = 0;

  slave.start();

  // Serial.println("SLAVE 10 READY");
}

// =====================================================

void loop()
{
  slave.poll(holdingRegs, 2);

  com = (int16_t)holdingRegs[0];
  // Serial.println(encoderTicks);

  switch(armState)
  {
    // ==========================================
    case IDLE:
    {
      holdingRegs[1] = 0;

      if(com == 1)
      {
        armState = GRIPPING;
      }
    }
    break;

    // ==========================================
    case GRIPPING:
    {
      holdingRegs[1] = 1;
      PID(-450);
    }
    break;

    // ==========================================
    case FINISHED:
    {
      stopMotor();
      holdingRegs[1] = 2;

      com = 0;
      armState = IDLE;
    }
    break;
  }
}
