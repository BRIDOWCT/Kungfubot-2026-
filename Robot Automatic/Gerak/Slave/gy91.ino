#include <Wire.h>
#include <MPU6050.h>

#define RS485_DIR 4

MPU6050 mpu;

float yaw = 0;

unsigned long lastTime;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  pinMode(RS485_DIR, OUTPUT);

  digitalWrite(RS485_DIR, LOW);

  mpu.initialize();

  lastTime = millis();

  Serial.println("IMU READY");
}

void loop() {

  readGyro();

  sendYaw();

  debugIMU();

  delay(10);
}

void readGyro() {

  int16_t gx, gy, gz;

  mpu.getRotation(&gx, &gy, &gz);

  float gyroZ = gz / 131.0;

  float dt =
    (millis() - lastTime) / 1000.0;

  lastTime = millis();

  yaw += gyroZ * dt;
}

void sendYaw() {

  digitalWrite(RS485_DIR, HIGH);

  Serial.print("I,");

  Serial.println(yaw,2);

  delay(2);

  digitalWrite(RS485_DIR, LOW);
}

void debugIMU() {

  Serial.print("Yaw : ");

  Serial.println(yaw);
}
