#include <ModbusRtu.h>
#include <Servo.h>

Servo srv1; Servo srv2;

#define ID 6
#define TXEN 4

#define rpwm 5
#define lpwm 6

// naik
int pwmUp = 40;

// turun
int pwmDown = 40;

bool servoTrigger = false; bool servoState = false;

int store = 0; int servo = 0;

Modbus slave(ID, Serial, TXEN);

uint16_t au16data[2] = {0, 0};

void gripper() {
  if (servo == 1) {
    if (!servoTrigger) {
      servoState = !servoState;
      servoTrigger = true;
      if (servoState) {
        srv1.write(10);
        srv2.write(50);
      }
      else {
        srv1.write(70);
        srv2.write(120);
      }
    }
  }

  else {

    servoTrigger = false;
  }
}
void storageMotor() {
  if (store == 1) {
    analogWrite(rpwm, pwmUp);
    analogWrite(lpwm, 0);
  }

  else if (store == 2) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, pwmDown);
  }
  else {

    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // MODBUS START
  slave.start();

  // MOTOR
  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);

  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);

  // SERVO
  srv1.attach(9);
  srv2.attach(10);

  // POSISI AWAL
  srv1.write(70);
  srv2.write(120);
}

void loop() {
  slave.poll(au16data, 2);
  store = (int16_t)au16data[0];
  servo = (int16_t)au16data[1];
  storageMotor();
  gripper();
}
