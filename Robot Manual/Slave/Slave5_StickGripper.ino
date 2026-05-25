#include <ModbusRtu.h>
#include <Servo.h>

Servo srv1;
Servo srv2;

#define pinSensor 2

#define rpwm 5
#define lpwm 6

#define pwm1 9
#define pwm2 10

#define en 4

volatile long pulses = 0;

int pos = 0;
int slide = 0;

bool motorState = true;

bool lastPos1 = false;
bool lastPos2 = false;

bool servoState1 = false;
bool servoState2 = false;

uint16_t au16data[2] = {0, 0};

Modbus slave(5, Serial, en);

void pulseCounter() { //ganti pakai counter aja

  if (motorState) {
    pulses++;
  } 
  else {
    pulses--;
  }
}

void updateGripper() {
  pos = (int16_t)au16data[0];
  slide = (int16_t)au16data[1];

  if (pos == 1) {

    if (!lastPos1) {

      servoState1 = !servoState1;

      if (servoState1) {
        srv1.write(110);
      } 
      else {
        srv1.write(0);
      }
    }

    lastPos1 = true;
  } 
  else {

    lastPos1 = false;
  }

  if (pos == 2) {

    if (!lastPos2) {

      servoState2 = !servoState2;

      if (servoState2) {
        srv2.write(0);
      } 
      else {
        srv2.write(90);
      }
    }

    lastPos2 = true;
  } 
  else {

    lastPos2 = false;
  }

  if (slide == 1 ) {

    motorState = true;

    analogWrite(rpwm, 70);
    analogWrite(lpwm, 0);
  }


  else if (slide == 2 ) {

    motorState = false;

    analogWrite(rpwm, 0);
    analogWrite(lpwm, 70);
  }

  else {

    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}


void setup() {

  
  Serial.begin(115200);

  
  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);

  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);

  
  pinMode(pinSensor, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(pinSensor),
    pulseCounter,
    FALLING
  );

  
  srv1.attach(pwm1);
  srv2.attach(pwm2);

  srv1.write(0);
  srv2.write(90);

  
  pinMode(en, OUTPUT);

  slave.start();
}

void loop() {
  slave.poll(au16data, 2);

  updateGripper();
}
