#include <Servo.h>

constexpr pin_size_t PIN_M1 = 0;
constexpr pin_size_t PIN_M2 = 2;
constexpr pin_size_t PIN_M3 = 4;
constexpr pin_size_t PIN_M4 = 6;
Servo m1_esc, m2_esc, m3_esc, m4_esc;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  m1_esc.attach(PIN_M1);
  m2_esc.attach(PIN_M2);
  m3_esc.attach(PIN_M3);
  m4_esc.attach(PIN_M4);
}

void loop() {
/* Push the pull rod of throttle in the ESC to the highest position to make the ESC enter the setting mode and
then turn on the ESC.
*/
    static int throttle = 2000;
    if (Serial.available() > 0) {
        byte receivedByte = Serial.read();
        switch (receivedByte) {
            case 0x41: // Up arrow
                Serial.println("Up arrow key pressed");
                throttle = 2000;
                break;
            case 0x42: // Down arrow
                Serial.println("Down arrow key pressed");
                throttle = 1000;
                break;
            case 0x43: // Right arrow
                Serial.println("Right arrow key pressed");
                break;
            case 0x44: // Left arrow
                Serial.println("Left arrow key pressed");
                break;
            default:
                break;
        }
    }
    m1_esc.writeMicroseconds(throttle);
    m2_esc.writeMicroseconds(throttle);
    m3_esc.writeMicroseconds(throttle);
    m4_esc.writeMicroseconds(throttle);
}
