#define RS485_DIR 4

#define RPWM_LF 5
#define LPWM_LF 6

#define RPWM_RF 9
#define LPWM_RF 10

String buffer = "";

void setup() {

  Serial.begin(115200);

  pinMode(RS485_DIR, OUTPUT);

  pinMode(RPWM_LF, OUTPUT);
  pinMode(LPWM_LF, OUTPUT);

  pinMode(RPWM_RF, OUTPUT);
  pinMode(LPWM_RF, OUTPUT);

  digitalWrite(RS485_DIR, LOW);

  Serial.println("FRONT READY");
}

void loop() {

  while(Serial.available()) {

    char c = Serial.read();

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

  if(data.startsWith("B")) {

    int p1 = data.indexOf(',');
    int p2 = data.indexOf(',', p1 + 1);

    int lf = data.substring(p1 + 1, p2).toInt();
    int rf = data.substring(p2 + 1).toInt();

    motorDrive(RPWM_LF, LPWM_LF, lf);
    motorDrive(RPWM_RF, LPWM_RF, rf);

    Serial.print("LF : ");
    Serial.print(lf);

    Serial.print(" RF : ");
    Serial.println(rf);
  }
}

void motorDrive(int rpwm, int lpwm, int speedMotor) {

  speedMotor = constrain(speedMotor, -255, 255);

  if(speedMotor > 0) {

    analogWrite(rpwm, speedMotor);
    analogWrite(lpwm, 0);
  }

  else {

    analogWrite(rpwm, 0);
    analogWrite(lpwm, abs(speedMotor));
  }
}
