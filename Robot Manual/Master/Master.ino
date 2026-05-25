#include <ModbusRtu.h>
#include <Arduino.h>
#include <Ps3Controller.h>

#define EN_PIN 4

#define MAX_SPEED       250
#define BASE_SPEED      100
#define SPEED_INCREMENT 7
#define ACCEL_COEFF     0.3f

// Variabel global
int realSpeed = BASE_SPEED;
int lX = 0, lY = 0, rX = 0, rY = 0;
int gripState = 0;
int pg36State = 0;
int posisi = 0;
int slider = 0;
int lift = 0;
int store = 0;
int servo = 0;

int targetM[4] = {0, 0, 0, 0};
float currentM[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Setup MODBUS
uint16_t au16data[4];
Modbus master(0, Serial, EN_PIN);
modbus_t telegram;

// SLAVE ADDRESS 
#define MOTOR_F 1
#define MOTOR_B 2
#define GRIPER_F 3
#define LIFTER 4
#define STICK 5
#define STORE 6

// === FUNCTION DEFINITIONS ===
inline void readInputs();
inline void readInputsR1();
inline void updateMotorSpeeds();
inline void updateMotorSpeedsR1();
inline void stickGriper();
inline void boxGriper();
inline void boxLifter();
inline void storage();
void sendToSlave(uint8_t id, int val1);
void sendToSlave(uint8_t id, int val1, int val2);
void sendToSlave(uint8_t id, int val1, int val2, int val3);

//koneksi ps 3
void onConnect() {
  Serial.println("Connected.");
}
void onDisconnect() {
  Serial.println("Disconnected.");
}

void setup() {
  Serial.begin(115200);

  master.start();
  master.setTimeOut(20);

  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("e8:6b:ea:c3:e4:3a");
  // Ps3.begin("e4:65:b8:12:2c:46");
}

void loop() {
  if (Ps3.isConnected()) {
    if (Ps3.data.button.r1){
      // R1 mode functions
      boxGriper();
      boxLifter();
      Storage();
      readInputsR1();
      updateMotorSpeedsR1();

      // Sending to slaves
      sendToSlave(MOTOR_F, (int)currentM[0],(int)currentM[1]); 
      sendToSlave(MOTOR_B, (int)currentM[2],(int)currentM[3]);
      sendToSlave(GRIPER_F, gripState, pg36State);
      sendToSlave(LIFTER, lift);
      sendToSlave(STORE, store, servo);

    } else{
      // Normal mode functions
      stickGriper();
      readInputs();
      updateMotorSpeeds();

      // Sending to slaves
      sendToSlave(MOTOR_F, (int)currentM[0],(int)currentM[1]);
      sendToSlave(MOTOR_B, (int)currentM[2],(int)currentM[3]);
      sendToSlave(STICK, posisi , slider);
    }
  }
}

//input stick ps
inline void readInputs() {
  lX = map(Ps3.data.analog.stick.lx, -128, 127, -realSpeed, realSpeed);
  lY = map(Ps3.data.analog.stick.ly, -128, 127, realSpeed, -realSpeed);
  rX = -map(Ps3.data.analog.stick.rx, -128, 127, -realSpeed, realSpeed);
  rY = map(Ps3.data.analog.stick.ry, -128, 127, realSpeed, -realSpeed);

  //Update kecepatan
  if (Ps3.data.button.l1) {
    realSpeed = min(realSpeed + SPEED_INCREMENT, MAX_SPEED);
  } else if (Ps3.data.button.l2) {
    realSpeed = max(realSpeed - 60, BASE_SPEED);
  } 
}

// Input stick ps (R1)
inline void readInputsR1() {
  lX = map(Ps3.data.analog.stick.lx, -128, 127, -realSpeed, realSpeed);
  lY = map(Ps3.data.analog.stick.ly, -128, 127, realSpeed, -realSpeed);
  rX = -map(Ps3.data.analog.stick.rx, -128, 127, -realSpeed, realSpeed);
  rY = map(Ps3.data.analog.stick.ry, -128, 127, realSpeed, -realSpeed);
}

//Update kecepatan motor 
inline void updateMotorSpeeds() { 
  float vel = min(sqrt((float)(lX * lX + lY * lY)), (float)realSpeed);
  float arah = atan2(lY, lX);
  float putar = rX * 0.4f;

  targetM[0] = vel * sin(arah - 0.785f) + putar;
  targetM[1] = vel * sin(arah + 0.785f) - putar;
  targetM[2] = -vel * sin(arah - 2.356f) + putar;
  targetM[3] = -vel * sin(arah + 2.356f) - putar;

  for (int i = 0; i < 4; i++) {
    currentM[i] += (targetM[i] - currentM[i]) * ACCEL_COEFF;
    currentM[i] = constrain(currentM[i], -realSpeed, realSpeed);
  }
}

// Update Kecepatan motor (R1)
inline void updateMotorSpeedsR1() { 
  float vel = min(sqrt((float)(lX * lX + lY * lY)), (float)realSpeed);
  float arah = atan2(lY, lX);
  float putar = rX * 0.4f;

  targetM[0] = vel * sin(arah - 0.785f) + putar;
  targetM[1] = vel * sin(arah + 0.785f) - putar;
  targetM[2] = -vel * sin(arah - 2.356f) + putar;
  targetM[3] = -vel * sin(arah + 2.356f) - putar;

  for (int i = 0; i < 4; i++) {
    currentM[i] += (targetM[i] - currentM[i]) * ACCEL_COEFF;
    currentM[i] = constrain(currentM[i], -realSpeed, realSpeed);
  }
}

// Stick griper (R1)
inline void stickGriper(){
  if (Ps3.data.button.triangle){
    posisi = 1;
  } else if (Ps3.data.button.square){
    posisi = 2;
  } else {
    posisi = 0;
  }

  if (Ps3.data.button.right){
    slider = 1;
  } else if (Ps3.data.button.left){
    slider = 2;
  } else if (Ps3.data.button.cross){
    slider = 3;
  } else {
    slider = 0;
  }
}

// Box griper (R1)
inline void boxGriper(){
  // Griper
  if (Ps3.data.button.circle){
    gripState = 1;
  } else {
    gripState = 0;
  }

  // Extender
  if (Ps3.data.button.up){
    pg36State = 2;
  } else if (Ps3.data.button.down){
    pg36State = 3;
  } else {
    pg36State = 0;
  }
}

// Box lifter (R1)
inline void boxLifter(){
  if (Ps3.data.button.triangle){
    lift = 1;
  } else if (Ps3.data.button.cross){
    lift = 2;
  } else {
    lift = 0;
  } 
}

inline void Storage(){
  if (Ps3.data.button.l1){
    store = 1;
  } else if (Ps3.data.button.l2){
    store = 2;
  } else {
    store = 0;
  }

  if (Ps3.data.button.r2){
    servo = 1;
  } else {
    servo = 0;
  }
}

// Serial Communication MODBUS
void sendToSlave(uint8_t id, int val1) {
  au16data[0] = (uint16_t)val1;

  telegram.u8id = id;          
  telegram.u8fct = 16;        
  telegram.u16RegAdd = 0;      
  telegram.u16CoilsNo = 1;     
  telegram.au16reg = au16data; 

  master.query(telegram);

  master.poll();
}

void sendToSlave(uint8_t id, int val1, int val2) {
  au16data[0] = (uint16_t)val1;
  au16data[1] = (uint16_t)val2;

  telegram.u8id = id;          
  telegram.u8fct = 16;         
  telegram.u16RegAdd = 0;      
  telegram.u16CoilsNo = 2;     
  telegram.au16reg = au16data; 

  master.query(telegram);

  master.poll();
}

void sendToSlave(uint8_t id, int val1, int val2, int val3) {
  au16data[0] = (uint16_t)val1;
  au16data[1] = (uint16_t)val2;
  au16data[2] = (uint16_t)val3;

  telegram.u8id = id;          
  telegram.u8fct = 16;        
  telegram.u16RegAdd = 0;      
  telegram.u16CoilsNo = 3;    
  telegram.au16reg = au16data; 

  master.query(telegram);

  master.poll();
}
