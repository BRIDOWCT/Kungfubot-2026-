#include <ModbusRtu.h>

#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox;

#define EN_PIN     4
#define MASTER_ID  0

#define MIN_SPEED  20
#define MAX_SPEED  40
#define BASE_SPEED 21

uint16_t dataReadX[2];
uint16_t dataReadY[2];

// uint16_t dataReadIMU[6];

uint16_t frontMotorReg[2];
uint16_t rearMotorReg[2];

uint16_t lifterReg[3]; //lifter tenagha

uint16_t armScanReg[3];//gripper tengah

uint16_t lifterRegM[3]; //lifter tengah baruy

uint16_t lifterCmd[1];
uint16_t lifterRead[3];

// Modbus master
Modbus    master(MASTER_ID, Serial, EN_PIN);
modbus_t  telegram[9]; //2

float posX = 0;
float posY = 0;
float yaw = 0;
float scale = 1.0; //scalling

// Target Variable
float targetX = 47.0f;
float targetY = -40.0f;   

// PIDX
float KpX = 0.6;
float KiX = 0.0;
float KdX = 0.4;
float integralX = 0;
float prevErrorX = 0;

// PIDY
float KpY = 0.9;
float KiY = 0.0;
float KdY = 0.4;
float integralY = 0;
float prevErrorY = 0;

// Angular 
float KpAngular = 0.65;
float KiAngular = 0;
float KdAngular = 0.1;
float integralAngular = 0;
float prevErrorAngular = 0;

// Motor Speed Variable
float motorSpeed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// semua state
enum RobotState
{
  STATE_INIT,
  STATE_GERAK_X,
  STATE_GERAK_Y,
  STATE_KOREKSI_ROTASI,
  STATE_SCAN_OBJECT,
  STATE_LIFT_TENGAH_UP,
  STATE_LIFT_TENGAH_DOWN,
  STATE_LIFT_TENGAH,
  STATE_FINISH
};

RobotState state = STATE_INIT;


void resetVariable() {
  // Reset Integral
  integralX = 0;
  integralY = 0;
  integralAngular = 0;

  // Reset Previous Error
  prevErrorX = 0;
  prevErrorY = 0;
  prevErrorAngular = 0;
}


void odometriBiasa(float vx, float vy, float arah, float putar) {
  int16_t LF = vx - vy;
  int16_t RF = vx + vy;
  int16_t LB = vx + vy;
  int16_t RB = vx - vy;

  // Set Limit Max Speed
  LF = constrain(LF, -MAX_SPEED, MAX_SPEED);
  RF = constrain(RF, -MAX_SPEED, MAX_SPEED);
  LB = constrain(LB, -MAX_SPEED, MAX_SPEED);
  RB = constrain(RB, -MAX_SPEED, MAX_SPEED);

  // Set Minimal Limit Speed
  if (abs(LF) < MIN_SPEED && LF != 0) LF = (LF > 0) ? MIN_SPEED : -MIN_SPEED;
  if (abs(RF) < MIN_SPEED && RF != 0) RF = (RF > 0) ? MIN_SPEED : -MIN_SPEED;
  if (abs(LB) < MIN_SPEED && LB != 0) LB = (LB > 0) ? MIN_SPEED : -MIN_SPEED;
  if (abs(RB) < MIN_SPEED && RB != 0) RB = (RB > 0) ? MIN_SPEED : -MIN_SPEED;

  sendFrontMotor(LF, RF);
  sendRearMotor(LB, RB);
}

void mecanumMove(float vx, float vy)
{
  int16_t LF = vx - vy;
  int16_t RF = vx + vy;

  int16_t LB = vx + vy;
  int16_t RB = vx - vy;

  LF = constrain(LF, -120, 120);
  RF = constrain(RF, -120, 120);

  LB = constrain(LB, -120, 120);
  RB = constrain(RB, -120, 120);

  sendFrontMotor(LF, RF);
  sendRearMotor(LB, RB);
}

void stopAllMotor()
{
  sendFrontMotor(0, 0);
  sendRearMotor(0, 0);
}

float PID_X(float error)
{
  integralX += error;
  integralX = constrain(integralX, -100, 100);

  float derivative = error - prevErrorX;

  prevErrorX = error;

  float absError = fabs(error);
  scale = 1.0;

  if (absError < 90) scale = 0.2;
  if (absError < 50) scale = 0.2;
  if (absError < 20) scale = 0.25;
  if (absError < 10) scale = 0.1;

  // Hasil PID ke kecepatan
  float PID = KpX*error + KiX*integralX + KdX*derivative;
  PID = constrain(PID * scale, -MAX_SPEED, MAX_SPEED);

  return PID;
}

float PID_Y(float error)
{
  integralY += error;
  integralY = constrain(integralY, -100, 100); // sesuaikan limitnya


  float derivative = error - prevErrorY;

  prevErrorY = error;

  return
      KpY*error +
      KiY*integralY +
      KdY*derivative;
}

float PID_Angular(float error) {
  integralAngular += error;
  // integralAngular = constrain(integralAngular, -100, 100); // sesuaikan limitnya

  float derivative = error - prevErrorAngular;

  prevErrorAngular = error;

  float absError = fabs(error);
  scale = 1.0;

  // if (absError < 50) scale = 0.85;
  // if (absError < 30) scale = 0.6;
  // if (absError < 15) scale = 0.30;
  if (absError < 10) scale = 0.2;

  // Hasil PID ke kecepatan
  float PID = KpX*error + KiX*integralX + KdX*derivative;
  PID = constrain(PID * scale, -MAX_SPEED, MAX_SPEED);

  return PID;
}


void sendArmCommand(uint16_t cmd)
{
  armScanReg[0] = cmd;

  telegram[6].u8id = 10;
  telegram[6].u8fct = 16;

  telegram[6].u16RegAdd = 0;
  telegram[6].u16CoilsNo = 1;

  telegram[6].au16reg = armScanReg;

  master.query(telegram[6]);

  while(master.getState()!=COM_IDLE)
    master.poll();
}

bool readSlave10() //salve gripper
{
  telegram[7].u8id = 10;
  telegram[7].u8fct = 3;

  telegram[7].u16RegAdd = 0;
  telegram[7].u16CoilsNo = 3;

  telegram[7].au16reg = armScanReg;

  master.query(telegram[7]);

  unsigned long timeout = millis();

  while(master.getState()!=COM_IDLE)
  {
    master.poll();

    if(millis()-timeout > 300)
      return false;
  }

  return true;
}


void sendLiftCommand(uint16_t cmd)
{
  lifterRegM[0] = cmd;

  telegram[8].u8id = 11;
  telegram[8].u8fct = 16;

  telegram[8].u16RegAdd = 0;
  telegram[8].u16CoilsNo = 1;

  telegram[8].au16reg = lifterRegM;

  master.query(telegram[8]);

  while(master.getState()!=COM_IDLE)
    master.poll();
}
bool readSlave11()
{
  telegram[9].u8id = 11;
  telegram[9].u8fct = 3;

  telegram[9].u16RegAdd = 0;
  telegram[9].u16CoilsNo = 3;

  telegram[9].au16reg = lifterRegM;

  master.query(telegram[9]);

  unsigned long timeout = millis();

  while(master.getState()!=COM_IDLE)
  {
    master.poll();

    if(millis()-timeout > 300)
      return false;
  }

  return true;
}
bool readSlaveX()
{
  telegram[0].u8id       = 1;
  telegram[0].u8fct      = 3;      // Read Holding Register
  telegram[0].u16RegAdd  = 0;      // Mulai register 0
  telegram[0].u16CoilsNo = 2;      // Baca 2 register
  telegram[0].au16reg    = dataReadX;

  master.query(telegram[0]);

  unsigned long timeout = millis();

  while (master.getState() != COM_IDLE)
  {
    master.poll();

    if (millis() - timeout > 500)
    {
      Serial.println("Timeout komunikasi");
      return false;
    }
  }

  return true;
}

bool readSlaveY()
{
  telegram[1].u8id       = 2;
  telegram[1].u8fct      = 3;      // Read Holding Register
  telegram[1].u16RegAdd  = 0;      // Mulai register 0
  telegram[1].u16CoilsNo = 2;      // Baca 2 register
  telegram[1].au16reg    = dataReadY;

  master.query(telegram[1]);

  unsigned long timeout = millis();

  while (master.getState() != COM_IDLE)
  {
    master.poll();

    if (millis() - timeout > 500)
    {
      Serial.println("Timeout komunikasi");
      return false;
    }
  }

  return true;
}

void sendFrontMotor(int16_t LF, int16_t RF)
{
  frontMotorReg[0] = LF;
  frontMotorReg[1] = RF;

  telegram[3].u8id = 3;
  telegram[3].u8fct = 16;
  telegram[3].u16RegAdd = 0;
  telegram[3].u16CoilsNo = 2;
  telegram[3].au16reg = frontMotorReg;

  master.query(telegram[3]);

  while(master.getState()!=COM_IDLE)
    master.poll();
}

void sendRearMotor(int16_t LB, int16_t RB)
{
  rearMotorReg[0] = LB;
  rearMotorReg[1] = RB;

  telegram[2].u8id = 4;
  telegram[2].u8fct = 16;
  telegram[2].u16RegAdd = 0;
  telegram[2].u16CoilsNo = 2;
  telegram[2].au16reg = rearMotorReg;

  master.query(telegram[2]);

  while(master.getState()!=COM_IDLE)
    master.poll();
}

// void sendLifterCommand(uint16_t cmd)
// {
//   lifterCmd[0] = cmd;

//   telegram[8].u8id = 8;
//   telegram[8].u8fct = 16;

//   telegram[8].u16RegAdd = 0;
//   telegram[8].u16CoilsNo = 1;

//   telegram[8].au16reg = lifterCmd;

//   master.query(telegram[8]);

//   while(master.getState()!=COM_IDLE)
//     master.poll();
// }

// bool readLifter()
// {
//   telegram[5].u8id = 8;
//   telegram[5].u8fct = 3;

//   telegram[5].u16RegAdd = 0;
//   telegram[5].u16CoilsNo = 3;

//   telegram[5].au16reg = lifterRead;

//   master.query(telegram[5]);

//   unsigned long timeout = millis();

//   while(master.getState()!=COM_IDLE)
//   {
//     master.poll();

//     if(millis()-timeout > 300)
//       return false;
//   }

//   return true;
// }

// uint16_t getLiftStatus()
// {
//   return lifterRead[2];
// }



// // gerak lifter
// void setLifter(int16_t targetTick)
// {
//   lifterReg[0] = targetTick;

//   telegram[4].u8id = 8;
//   telegram[4].u8fct = 16;

//   telegram[4].u16RegAdd = 0;
//   telegram[4].u16CoilsNo = 1;

//   telegram[4].au16reg = lifterReg;

//   master.query(telegram[4]);

//   while(master.getState()!=COM_IDLE)
//     master.poll();
// }

// baca
// bool readLifter()
// {
//   telegram[4].u8id = 8;
//   telegram[4].u8fct = 3;

//   telegram[4].u16RegAdd = 0;
//   telegram[4].u16CoilsNo = 3;

//   telegram[4].au16reg = lifterReg;

//   master.query(telegram[4]);

//   unsigned long timeout = millis();

//   while(master.getState()!=COM_IDLE)
//   {
//     master.poll();

//     if(millis()-timeout > 300)
//       return false;
//   }

//   return true;
// }

// long getLiftTick()
// {
//   return (int16_t)lifterReg[1];
// }

// uint16_t getLiftStatus() //statuss
// {
//   return lifterReg[2];
// }



float convertToFloatX()
{
  union
  {
    float f;
    uint16_t reg[2];
  } data;

  data.reg[0] = dataReadX[0];
  data.reg[1] = dataReadX[1];

  return data.f;
}

float convertToFloatY()
{
  union
  {
    float f;
    uint16_t reg[2];
  } data;

  data.reg[0] = dataReadY[0];
  data.reg[1] = dataReadY[1];

  return data.f;
}


uint16_t readTOF()
{
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure,false);

  if(measure.RangeStatus != 4)
    return measure.RangeMilliMeter;

  return 8190;
}

uint16_t getArmStatus()
{
  return armScanReg[2];
}

uint16_t getLiftStatus()
{
  return lifterRegM[2];
}
void setup()
{
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);

  master.start();
  master.setTimeOut(200);

  if(!lox.begin()) //koneksi tof
{
  Serial.println("VL53L0X ERROR");

  while(1);
}

  // Serial.println("MODBUS MASTER AS5600");
}


void loop()
{

  readSlaveX();
  posX = convertToFloatX();

  readSlaveY();
  posY = convertToFloatY();
      
      
  // Read Encoder Data
  // readSlaveX();
  // readSlaveY();
  // // readSlaveIMU();

  // posX = convertToFloatX();           // regToFloat(dataReadX);
  // posY = convertToFloatY();           // regToFloat(dataReadY);

  switch(state)
  {
    case STATE_INIT:
    {
      stopAllMotor();

      // setGripper(90,90);
      // sendGripperCommand(1);

      delay(1000);

      state = STATE_GERAK_X; //STATE_GERAK_X ; //coba gerak poseidon
      break;
    }
   
    case STATE_GERAK_X:
    {
      
      // Error target
      float errorX = targetX - posX;
      // float rotationY = posY;

      // PID Encoder
      float vx = PID_X(errorX);
      // float vAngular = PID_Angular(rotationY);

      if (vx > 0) vx += BASE_SPEED;
      else vx -= BASE_SPEED;

      if(abs(errorX) < 4) // Treshold
      {
        stopAllMotor();
        delay(1000);

        resetVariable();
        state = STATE_SCAN_OBJECT; //STATE_GERAK_Y
      } else {
      
        odometriBiasa(vx, 0, 0, 0);
      }
      break;
    }
    
    // case STATE_GERAK_Y:
    // {
    //   // Error target
    //   float errorY = targetY - posY;

    //   // PID Encoder
    //   float vy = PID_Y(errorY);

    //   if (vy > 0) vy += BASE_SPEED;
    //   else vy -= BASE_SPEED;

    //   if(abs(errorY) < 2) // Treshold
    //   {
    //     stopAllMotor();
    //     delay(1000);

    //     resetVariable();
    //     state = STATE_FINISH;
    //   } else {
    //     odometriBiasa(0, vy, 0, 0);
    //   }
    //   break;
    // }

  case STATE_SCAN_OBJECT:
    {
      static bool startScan = false;
      static bool objectFound = false;

      if(!startScan)
      {
        sendArmCommand(1);

        startScan = true;
      }

      uint16_t jarak = readTOF();

      /*
      Serial.print("TOF = ");
      Serial.print(jarak);
      Serial.println(" mm");
      */

      if(jarak < 70 && !objectFound)
      {
        sendArmCommand(2);

        objectFound = true;
      }

      if(readSlave10())
      {
        if(getArmStatus() == 2)
        {
          startScan = false;
          objectFound = false;

          state = STATE_FINISH;
        }
      }
    }
    break;

    
    case STATE_LIFT_TENGAH_UP:
      {
        static bool startLift = false;

        if(!startLift)
        {
          sendLiftCommand(1);

          startLift = true;
        }

        if(readSlave11())
        {
          if(getLiftStatus() == 2)
          {
            startLift = false;

            state = STATE_LIFT_TENGAH_DOWN;
          }
        }
      }
      break;


      case STATE_LIFT_TENGAH_DOWN:
{
          static bool startLift = false;

          if(!startLift)
          {
            sendLiftCommand(2);

            startLift = true;
          }

          if(readSlave11())
          {
            if(getLiftStatus() == 2)
            {
              startLift = false;

              state = STATE_FINISH;
            }
          }
        }
        break;
                
      //   case STATE_LIFT_TENGAH_UP:
      //     {
      //         static bool commandSent = false;

      //         if(!commandSent)
      //         {
      //             setLifter(1000);   // sesuaikan arah encoder
      //             commandSent = true;

      //             // Serial.println("upppppp");
      //         }

      //         if(readLifter())
      //         {
      //             if(getLiftStatus() == 2)
      //             {
      //                 commandSent = false;

      //                 state = STATE_LIFT_TENGAH_DOWN;
      //             }
      //         }
      //     }
      //     break;

      // case STATE_LIFT_TENGAH_DOWN:
      //   {
      //       static bool commandSent = false;

      //       if(!commandSent)
      //       {
      //           setLifter(0);

      //           commandSent = true;

      //           // Serial.println("downnnn");
      //       }

      //       if(readLifter())
      //       {
      //           if(getLiftStatus() == 2)
      //           {
      //               commandSent = false;

      //               state = STATE_FINISH;
      //           }
      //       }
      //   }
      //   break;

    case STATE_FINISH:
    {
      stopAllMotor();  
      break;
    }
  }

  delay(20);

  // yaw = convertToFloatIMU();       //regToFloat(&dataReadIMU[0]);

  // if (readSlaveX())
  // {
  //   float distance = convertToFloatX();

  //   Serial.print("Slave ID : 1");
  //   Serial.print(" | Reg0 : ");
  //   Serial.print(dataReadX[0]);

  //   Serial.print(" | Reg1 : ");
  //   Serial.print(dataReadX[1]);

  //   Serial.print(" | Distance : ");
  //   Serial.println(distance, 3);
  // }

  // if (readSlaveY())
  //   {
  //     float distance = convertToFloatY();

  //     Serial.print("Slave ID : 2");
  //     Serial.print(" | Reg0 : ");
  //     Serial.print(dataReadY[0]);

  //     Serial.print(" | Reg2 : ");
  //     Serial.print(dataReadY[1]);

  //     Serial.print(" | Distance : ");
  //     Serial.println(distance, 3);
  //   }
}
