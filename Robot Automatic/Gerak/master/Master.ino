#define RS485_EN 4

void setup() {

  Serial.begin(115200);

  pinMode(RS485_EN, OUTPUT);

  digitalWrite(RS485_EN, LOW);
}

void loop() {

  float angle1 = RequestData("1");

  delay(10);

  float angle2 = RequestData("2");

  delay(10);

  Serial.print("AS5600_1 : ");
  Serial.print(angle1);

  Serial.print(" || AS5600_2 : ");
  Serial.println(angle2);

  delay(100);
}

float RequestData(String address) {

  String received = "";

  digitalWrite(RS485_EN, HIGH);

  Serial.print(address);
  Serial.print("\n");

  Serial.flush();

  delay(2);

  digitalWrite(RS485_EN, LOW);

  unsigned long timeout = millis();

  while (millis() - timeout < 100) {

    while (Serial.available()) {

      char c = Serial.read();

      if (c == '\n') {

        return received.toFloat();
      }

      received += c;
    }
  }

  return 0;
}
