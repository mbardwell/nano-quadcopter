// https://www.instructables.com/ESC-Programming-on-Arduino-Hobbyking-ESC/
#include <Servo.h>
#include <Wire.h>

constexpr unsigned M1_PIN_CONTROL = 0;
constexpr unsigned M2_PIN_CONTROL = 2;
constexpr unsigned M3_PIN_CONTROL = 4;
constexpr unsigned M4_PIN_CONTROL = 6;
Servo M4_ESC, M1_ESC, M3_ESC, M2_ESC;

constexpr byte I2C_SDA = 8;
constexpr byte I2C_SCL = 9;
constexpr byte I2C_ADDRESS_MPU = 0x68;

const unsigned MAX_RUNS = 1;
float RateRoll, RatePitch, RateYaw;

// Forward declarations
bool gyro_signals(void);

void setup() {
  Serial.begin(115200);

  // Setup I2C
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(I2C_ADDRESS_MPU); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);

  // Setup motors
  // M1_ESC.attach(M1_PIN_CONTROL);
  // M2_ESC.attach(M2_PIN_CONTROL);
  // M3_ESC.attach(M3_PIN_CONTROL);
  // M4_ESC.attach(M4_PIN_CONTROL);
  // Serial.begin(115200);
  // M1_ESC.writeMicroseconds(0);
  // M2_ESC.writeMicroseconds(0);
  // M3_ESC.writeMicroseconds(0);
  // M4_ESC.writeMicroseconds(0);
  // delay(5000);
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

  static bool change = true;
  change = gyro_signals();

  if (change) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(RateRoll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(RatePitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(RateYaw);
    delay(50);
  }
  else {
    Serial.println("Error: Read");
    delay(500);
  }

  // if (runs >= MAX_RUNS) {
  //   Serial.println("Not running");
  //   M1_ESC.writeMicroseconds(0);
  //   M2_ESC.writeMicroseconds(0);
  //   M3_ESC.writeMicroseconds(0);
  //   M4_ESC.writeMicroseconds(0);
  //   delay(1000);
  //   return;
  // }

  // if ((millis() - hold) > AUTO_START_EVERY_MS) {
  //   for (value = AUTO_MIN_VALUE; value < AUTO_MAX_VALUE; value++) {
  //     M1_ESC.writeMicroseconds(value);
  //     M4_ESC.writeMicroseconds(value);
  //     M3_ESC.writeMicroseconds(value);
  //     M2_ESC.writeMicroseconds(value);
  //     delay(AUTO_STAGE_SIT_MS);
  //   }
  //   for (value = AUTO_MAX_VALUE; value > AUTO_MIN_VALUE; value--) {
  //     Serial.println(value);
  //     M1_ESC.writeMicroseconds(value);
  //     M2_ESC.writeMicroseconds(value);
  //     M3_ESC.writeMicroseconds(value);
  //     M4_ESC.writeMicroseconds(value);
  //     delay(AUTO_STAGE_SIT_MS);
  //   }

  //   M1_ESC.writeMicroseconds(0);
  //   M2_ESC.writeMicroseconds(0);
  //   M3_ESC.writeMicroseconds(0);
  //   M4_ESC.writeMicroseconds(0);

  //   runs++;
  //   hold = millis();
  // }
  
  // if ((millis() - hold_print) > 1000) {
  //   Serial.print("Holding: ");
  //   Serial.print(millis() - hold);
  //   Serial.println("ms");
  //   hold_print = millis();
  // }
}


bool gyro_signals(void) {
  const float DEG_TO_LSB = 65.5;
  static int16_t GyroX = 0;
  static int16_t GyroY = 0;
  static int16_t GyroZ = 0;
  Wire.beginTransmission(I2C_ADDRESS_MPU);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(I2C_ADDRESS_MPU);
  Wire.write(0x1B); 
  Wire.write(0x8); 
  Wire.endTransmission(); 
  Wire.beginTransmission(I2C_ADDRESS_MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS_MPU,6);
  if (Wire.available() == 6) {
    GyroX = Wire.read()<<8 | Wire.read();
    GyroY = Wire.read()<<8 | Wire.read();
    GyroZ = Wire.read()<<8 | Wire.read();
    
    RateRoll = static_cast<float>(GyroX) / DEG_TO_LSB;
    RatePitch = static_cast<float>(GyroY) / DEG_TO_LSB;
    RateYaw = static_cast<float>(GyroZ) / DEG_TO_LSB;
    
    return true;
  }
  return false;
}
