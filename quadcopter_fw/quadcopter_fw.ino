// https://www.instructables.com/ESC-Programming-on-Arduino-Hobbyking-ESC/
#include <Servo.h>

const unsigned M1_PIN_CONTROL = 0;
const unsigned M2_PIN_CONTROL = 2;
const unsigned M3_PIN_CONTROL = 4;
const unsigned M4_PIN_CONTROL = 6;
const unsigned MAX_RUNS = 1;
Servo M4_ESC, M1_ESC, M3_ESC, M2_ESC;

void setup() {
  M1_ESC.attach(M1_PIN_CONTROL);
  M2_ESC.attach(M2_PIN_CONTROL);
  M3_ESC.attach(M3_PIN_CONTROL);
  M4_ESC.attach(M4_PIN_CONTROL);
  Serial.begin(115200);
  M1_ESC.writeMicroseconds(0);
  M2_ESC.writeMicroseconds(0);
  M3_ESC.writeMicroseconds(0);
  M4_ESC.writeMicroseconds(0);
  delay(5000);
}

void loop() {
  const int AUTO_START_EVERY_MS = 15000;
  const int AUTO_STAGE_SIT_MS = 5;
  const int AUTO_MIN_VALUE = 1200;
  const int AUTO_MAX_VALUE = 1500;
  static int hold = millis();
  static int hold_print = millis();
  static int runs = 0;
  static int value = 0;

  if (runs >= MAX_RUNS) {
    Serial.println("Not running");
    M1_ESC.writeMicroseconds(0);
    M2_ESC.writeMicroseconds(0);
    M3_ESC.writeMicroseconds(0);
    M4_ESC.writeMicroseconds(0);
    delay(1000);
    return;
  }

  if ((millis() - hold) > AUTO_START_EVERY_MS) {
    for (value = AUTO_MIN_VALUE; value < AUTO_MAX_VALUE; value++) {
      M1_ESC.writeMicroseconds(value);
      M4_ESC.writeMicroseconds(value);
      M3_ESC.writeMicroseconds(value);
      M2_ESC.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }
    for (value = AUTO_MAX_VALUE; value > AUTO_MIN_VALUE; value--) {
      Serial.println(value);
      M1_ESC.writeMicroseconds(value);
      M2_ESC.writeMicroseconds(value);
      M3_ESC.writeMicroseconds(value);
      M4_ESC.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }

    M1_ESC.writeMicroseconds(0);
    M2_ESC.writeMicroseconds(0);
    M3_ESC.writeMicroseconds(0);
    M4_ESC.writeMicroseconds(0);

    runs++;
    hold = millis();
  }
  
  if ((millis() - hold_print) > 1000) {
    Serial.print("Holding: ");
    Serial.print(millis() - hold);
    Serial.println("ms");
    hold_print = millis();
  }
}
