/**
 * @file esc-settings.ino
 * @brief ESC (Electronic Speed Controller) settings for a nano quadcopter.
 *
 * This program initializes and controls four ESCs using the Servo library.
 * It sets up serial communication to receive commands for adjusting the throttle.
 *
 * @details Use this script to apply the following non-default settings:
 * - Angle of entrance: High
 * - Startup of motor: Accelerated startup
 * - PWM frequency: 16Hz
 * - Protection mode: Cutoff
 */

#include <Servo.h>

constexpr pin_size_t PIN_M1 = 0; ///< Pin for Motor 1 ESC
constexpr pin_size_t PIN_M2 = 2; ///< Pin for Motor 2 ESC
constexpr pin_size_t PIN_M3 = 4; ///< Pin for Motor 3 ESC
constexpr pin_size_t PIN_M4 = 6; ///< Pin for Motor 4 ESC
Servo m1_esc, m2_esc, m3_esc, m4_esc; ///< Servo objects for each ESC

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  m1_esc.attach(PIN_M1);
  m2_esc.attach(PIN_M2);
  m3_esc.attach(PIN_M3);
  m4_esc.attach(PIN_M4);
}

/**
 * @brief Main loop function to read serial input and adjust throttle accordingly.
 *
 * The throttle is set to 2000 microseconds when the up arrow key is pressed,
 * and to 1000 microseconds when the down arrow key is pressed.
 * The throttle value is sent to all four ESCs.
 */
void loop() {
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
