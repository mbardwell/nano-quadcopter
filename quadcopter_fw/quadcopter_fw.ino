// https://www.instructables.com/ESC-Programming-on-Arduino-Hobbyking-ESC/
#include <Servo.h>

// All w.r.t the quadcopter upright and snout facing away from you
const unsigned FR_PIN_CONTROL = 11;
const unsigned FL_PIN_CONTROL = 4;
const unsigned BL_PIN_CONTROL = 20;
const unsigned BR_PIN_CONTROL = 0;
const unsigned MAX_RUNS = 1;
Servo FL_ESC, FR_ESC, BL_ESC, BR_ESC;

void setup() {
  FR_ESC.attach(FR_PIN_CONTROL);
  FL_ESC.attach(FL_PIN_CONTROL);
  BL_ESC.attach(BL_PIN_CONTROL);
  BR_ESC.attach(BR_PIN_CONTROL);
  Serial.begin(115200);
  FR_ESC.writeMicroseconds(0);
  FL_ESC.writeMicroseconds(0);
  BL_ESC.writeMicroseconds(0);
  BR_ESC.writeMicroseconds(0);
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
    FR_ESC.writeMicroseconds(0);
    FL_ESC.writeMicroseconds(0);
    BL_ESC.writeMicroseconds(0);
    BR_ESC.writeMicroseconds(0);
    delay(1000);
    return;
  }

  int FR_CATCH_UP_DELAY = 5;
  if ((millis() - hold) > AUTO_START_EVERY_MS) {
    int fr_catch_up_counter = 0;
    for (value = AUTO_MIN_VALUE; value < AUTO_MAX_VALUE; value++) {
      Serial.println(value);
      if (fr_catch_up_counter++ < FR_CATCH_UP_DELAY) {
        Serial.println("Catching up front right motor");
        FR_ESC.writeMicroseconds(value);
        delay(AUTO_STAGE_SIT_MS);
        continue;
      }
      FL_ESC.writeMicroseconds(value);
      BL_ESC.writeMicroseconds(value);
      BR_ESC.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }
    for (value = AUTO_MAX_VALUE; value > AUTO_MIN_VALUE; value--) {
      Serial.println(value);
      FR_ESC.writeMicroseconds(value);
      FL_ESC.writeMicroseconds(value);
      BL_ESC.writeMicroseconds(value);
      BR_ESC.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }

    FR_ESC.writeMicroseconds(0);
    FL_ESC.writeMicroseconds(0);
    BL_ESC.writeMicroseconds(0);
    BR_ESC.writeMicroseconds(0);

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
