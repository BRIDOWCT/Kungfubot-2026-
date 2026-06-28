#include <ModbusRtu.h>
#include <Servo.h>

#define SLAVE_ID 7
#define EN_PIN   4

#define SERVO1_PIN 10
#define SERVO2_PIN 9

Modbus slave(SLAVE_ID, Serial, EN_PIN);

uint16_t holdingRegs[1];

Servo angkat;
Servo grip;

void setup()
{
  Serial.begin(115200);

  angkat.attach(SERVO1_PIN);
  grip.attach(SERVO2_PIN);

  // Posisi awal
  angkat.write(10);
  grip.write(70);

  slave.start();

  Serial.println("Servo Slave Ready");
}

void loop()
{
  slave.poll(holdingRegs, 1);

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 500)
  {
    lastPrint = millis();

    Serial.print("CMD = ");
    Serial.println(holdingRegs[0]);
  }

  if (holdingRegs[0] == 1)
  {
    Serial.println("grip");

    grip.write(0);
    delay(300);

    angkat.write(90);

    holdingRegs[0] = 0;
  }

  else if (holdingRegs[0] == 2)
  {
    Serial.println("buka");

    angkat.write(0);
    delay(300);

    grip.write(0);

    holdingRegs[0] = 0;
  }
}
