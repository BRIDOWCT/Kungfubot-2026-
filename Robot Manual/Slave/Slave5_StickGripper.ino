#include <ModbusRtu.h>
#include <Servo.h>

Servo srv1;
Servo srv2;

const int pinSensor = 2;
volatile long pulses = 0;

// Pins
#define rpwm 5
#define lpwm 6
#define pwm1 9
#define pwm2 10
#define en 4 

int pos = 0;
int slide = 0;

int target = -150;
float kp = 1.5;
float ki = 0.0;
float kd = 0.2;

int error = 0;
int last_error = 0;
int derivative = 0;
int integral = 0;

bool motorState = false;

// bool motorLock = false;
// bool motorStatus = false;

bool lastPos1 = false;
bool servoState1 = false; 



int angle = 0;

// Data Modbus
uint16_t au16data[2] = {0,0};

bool lastPos2 = false;
bool lastPos3 = false;

unsigned long lastMove = 0;

// Inisiasi modbus slave
Modbus slave(5, Serial, en);

void pulseCounter(){
  if (motorState == true){
    pulses++;
  } 
  else {
    pulses--;
  }

}

void slider(int pulse, int target){
  error = target - pulse;
  integral += error;
  derivative = error - last_error;

  float output = kp*error + ki*integral + kd*derivative;
  last_error = error;

  output = constrain(output, -70, 70);
  Serial.print("Output: ");
  Serial.println(output); 

  if (abs(error) < 2){
    integral = 0;
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
    // Serial.println("Ga Gerak");
  } else {
    if (output > 0){
      motorState = true;
      interrupts();
      analogWrite(rpwm, output);
      analogWrite(lpwm, 0);
      // Serial.println("Maju");
    } else {
      motorState = false;
      interrupts();
      analogWrite(rpwm, 0);
      analogWrite(lpwm, abs(output));
      // Serial.println("Mundur");
    }
  }
}

void updateGriper(){
  pos = (int16_t)au16data[0];
  slide = (int16_t)au16data[1];

  if (pos == 1){
    if (!lastPos1){
      servoState1 = !servoState1;

      if (servoState1){
        srv1.write(220);
      } else {
        srv1.write(0);
      }
    }
    lastPos1 = true;
  } else {
    lastPos1 = false;
  }

//   if (pos == 2){
//     if(!lastPos2){
//         angle++;

//         if(angle > 90)
//             angle = 90;

//         srv2.write(angle);
//     }

//     lastPos2 = true;
// }
// else{
//     lastPos2 = false;
// } 

if (pos == 2) {
    if (millis() - lastMove >= 100) {
        // angle++;

        angle+=2;
        angle = constrain(angle, 0, 90);
        srv2.write(angle);
        lastMove = millis();
    }
}

if (pos == 3) {
    if (millis() - lastMove >= 100) {
        // angle--;

        angle-=2;
        angle = constrain(angle, 0, 90);
        srv2.write(angle);
        lastMove = millis();
    }
}

// if(pos == 3){
//     if(!lastPos3){
//         angle--;

//         if(angle < 0)
//             angle = 0;

//         srv2.write(angle);
//     }

//     lastPos3 = true;
// }
// else{
//     lastPos3 = false;
// }
    
  if (pulses > 50){

    if (slide == 1){
      // motorLock = false;
      // motorStatus = false;
      // Serial.println("TESTTTTTTTTTT");
      motorState = false;
      interrupts();
      analogWrite(rpwm, 0);
      analogWrite(lpwm, 70);
    } 
    else {
      analogWrite(rpwm, 0);
      analogWrite(lpwm, 0);
    }
  } 
  else if (pulses < -70){

    if (slide == 2){
      // Serial.println("WOIIIIIIIIII");
      // motorLock = false;
      // motorStatus = false;
      motorState = true;
      interrupts();
      analogWrite(rpwm, 70);
      analogWrite(lpwm, 0);
    }
    else {
      analogWrite(rpwm,0);
      analogWrite(lpwm,0);
    }
  }
  else {
    // Serial.println("HALOOOOOOOOO");
    // motorLock = false;
    // motorStatus = false;
    
    if (slide == 2){
      motorState = true;
      interrupts();
      analogWrite(rpwm, 70);
      analogWrite(lpwm, 0);
    } 
    else if (slide == 1){
      motorState = false;
      interrupts();
      analogWrite(rpwm, 0);
      analogWrite(lpwm, 70);
    }
    else {
      analogWrite(rpwm, 0);
      analogWrite(lpwm, 0);
    }
  }

  // if (slide == 3){
  //   if (!motorLock){
  //     motorStatus = !motorStatus;
  //     motorLock = true;
      
  //   }
    
  // }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);

  analogWrite(rpwm, 0);
  analogWrite(lpwm, 0);

  pinMode(pinSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinSensor), pulseCounter, FALLING);

  srv1.attach(pwm1);
  srv2.attach(pwm2);

  srv1.write(0);
  srv2.write(0);

  pinMode(en, OUTPUT);
  slave.start();

}

void loop() {
  // put your main code here, to run repeatedly:
  slave.poll(au16data, 2);
  // Serial.print(pos);
  // Serial.print("    ");
  // Serial.println(angle);

  updateGriper();

  // if (motorStatus){
  //   slider(pulses, target);
  // }

  // Serial.print("pulse: ");
  // Serial.println(pulses);

}
