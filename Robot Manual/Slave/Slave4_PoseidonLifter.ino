#include <ModbusRtu.h>

#define rpwm 6
#define lpwm 5
#define TXEN 4
#define ENA 3
#define ENB 2

int com = 0;

float kp = 2.0;
float ki = 0.0;
float kd = 0.2;

float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float last_pulse = 0;

// int sampling = 20;
// float dt = sampling/1000;

// int liftState = 0;

volatile long pulse = 0;
// int count = 0;
unsigned long holdTimer = 0;
bool holdInit = false;

Modbus slave(4, Serial, TXEN);
uint16_t au16data[1] = {0};

void enc(){
  if(digitalRead(ENB)==HIGH){
    pulse++;
  } else{
    pulse--;
  }
}

void PID(float target){

  error = target - pulse;
  integral += error;
  derivative = (error - last_error);

  float output = kp*error + ki*integral + kd*derivative;

  last_error = error;

  output = constrain(output, -150, 50);
  // Serial.print("output: ");
  // Serial.println(output);
  // Serial.print("error: ");
  // Serial.println(error);


  if (!(abs(error) < 2)){
    if (output > 0){
      analogWrite(rpwm, 0);
      analogWrite(lpwm, output);
        
    } 
    else {
      analogWrite(rpwm, abs(output));
      analogWrite(lpwm, 0);
      // Serial.println("TESTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT");
    }
      
  } 
  else {
    integral = 0;
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

// void select(float target){

//   if (com == 1){

//     liftState = 1;

//   } 
//   else if (com == 2){

//     liftState = 2;

//   } 
//   else if (com == 3){

//     liftState = 3;

//   }

//   if (liftState == 1) {
//     lifter(-50);
//   } 
//   else if (liftState == 2) {
//     lifter(target);
//   } 
//   else if (liftState == 3) {
//     lifter(target-160);
//   } 
// }

void lift(){

  if (pulse > -10){

    if (com == 1){

      holdInit = false;

      analogWrite(rpwm, 100);
      analogWrite(lpwm, 0);

    } 
    else {

      if (!holdInit){
        holdTimer = millis();
        last_pulse = pulse;
        holdInit = true;
      }

      PID(last_pulse);
    }
  } 
  
  else if (pulse < -420){

    if (com == 2){

      holdInit = false;

      analogWrite(rpwm, 0);
      analogWrite(lpwm, 75);

    } 
    else {

      if (!holdInit){
        holdTimer = millis();
        last_pulse = pulse;
        holdInit = true;
      }

      PID(last_pulse);
    }
  }
  
  else {

    if (com == 1){

      holdInit = false;

      analogWrite(rpwm, 100);
      analogWrite(lpwm, 0);
    }

    else if (com == 2){

      holdInit = false;

      analogWrite(rpwm, 0);
      analogWrite(lpwm, 75);
    }

    else {

      if (!holdInit){
        holdTimer = millis();
        last_pulse = pulse;
        holdInit = true;
      }

      PID(last_pulse);
    }
  }
}

// void lift(){
//   if (pulse > -10){
//     if (com == 1){
//       count = 0;
//       analogWrite(rpwm, 100);
//       analogWrite(lpwm, 0);
//     } else {
//       count++;
//       if (count == 1){
//         last_pulse = pulse;
//       }
//       PID(last_pulse);
//     }
//   } 
//   else if (pulse < -420){
//     if (com == 2){
//       count = 0;
//       analogWrite(rpwm, 0);
//       analogWrite(lpwm, 75);
//     } else {
//       count++;
//       if (count == 1){
//         last_pulse = pulse;
//       }
//       PID(last_pulse);
//     }
//   }
//   else {
//     if (com == 1){
//       count = 0;
//       analogWrite(rpwm, 100);
//       analogWrite(lpwm, 0);
//     }
//     else if (com == 2){
//       count = 0;
//       analogWrite(rpwm, 0);
//       analogWrite(lpwm, 75);
//     }
//     else {
//       count++;
//       if (count == 1){
//         last_pulse = pulse;
//       }
//       PID(last_pulse);
//     }
//   }
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  slave.start();

  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);
  pinMode(ENA, INPUT_PULLUP);
  pinMode(ENB, INPUT_PULLUP);

  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);

  attachInterrupt(digitalPinToInterrupt(ENA), enc, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  slave.poll(au16data, 1);
  com = (int16_t)au16data[0];

  interrupts(); 
  Serial.print("com: ");
  Serial.println(com);

  lift();
}
