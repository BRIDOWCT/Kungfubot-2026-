#include <Wire.h>

#define RXD2 16
#define TXD2 17

#define RS485_DIR 4

enum RobotState {
  MOVE_FORWARD,
  STOP_1,
  STRAFE_LEFT,
  STOP_ALL
};

RobotState state = MOVE_FORWARD;

float posX = 0;
float posY = 0;
float yaw = 0;

float targetForward = 1.0;
float targetLeft = 2.0;

String buffer = "";

void setup() {

  Serial.begin(115200);

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(RS485_DIR, OUTPUT);

  digitalWrite(RS485_DIR, LOW);

  Serial.println("MASTER READY");
}

void loop() {

  receiveData();

  stateRobot();

  debugMonitor();

  delay(20);
}

void stateRobot() {

  switch(state) {

    case MOVE_FORWARD:

      if(posY < targetForward) {

        mecanumMove(120,120,120,120);

      } else {

        stopAll();

        delay(1000);

        state = STRAFE_LEFT;
      }

    break;

    case STRAFE_LEFT:

      if(abs(posX) < targetLeft) {

        mecanumMove(-120,120,120,-120);

      } else {

        stopAll();

        state = STOP_ALL;
      }

    break;

    case STOP_ALL:

      stopAll();

    break;
  }
}

void mecanumMove(int lf, int rf, int lb, int rb) {

  float correction = yaw * 5.0;

  lf -= correction;
  rf += correction;

  lb -= correction;
  rb += correction;

  sendFront(lf, rf);

  delay(2);

  sendBack(lb, rb);
}

void stopAll() {

  sendFront(0,0);

  delay(2);

  sendBack(0,0);
}

void sendFront(int lf, int rf) {

  digitalWrite(RS485_DIR, HIGH);

  String data =
    "F," +
    String(lf) + "," +
    String(rf) + "\n";

  Serial2.print(data);

  delay(2);

  digitalWrite(RS485_DIR, LOW);
}

void sendBack(int lb, int rb) {

  digitalWrite(RS485_DIR, HIGH);

  String data =
    "B," +
    String(lb) + "," +
    String(rb) + "\n";

  Serial2.print(data);

  delay(2);

  digitalWrite(RS485_DIR, LOW);
}

void receiveData() {

  while(Serial2.available()) {

    char c = Serial2.read();

    if(c == '\n') {

      parsing(buffer);

      buffer = "";
    }
    else {

      buffer += c;
    }
  }
}

void parsing(String data) {

  if(data.startsWith("X")) {

    int p1 = data.indexOf(',');

    posX = data.substring(p1 + 1).toFloat();
  }

  else if(data.startsWith("Y")) {

    int p1 = data.indexOf(',');

    posY = data.substring(p1 + 1).toFloat();
  }

  else if(data.startsWith("I")) {

    int p1 = data.indexOf(',');

    yaw = data.substring(p1 + 1).toFloat();
  }
}

void debugMonitor() {

  Serial.print("X : ");
  Serial.print(posX);

  Serial.print(" | Y : ");
  Serial.print(posY);

  Serial.print(" | Yaw : ");
  Serial.print(yaw);

  Serial.print(" | State : ");
  Serial.println(state);
}
