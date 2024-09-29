#include <Adafruit_BMP280.h>
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

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
const unsigned MAX_RUNS = 1;
float RateRoll, RatePitch, RateYaw;
sensors_event_t temp_event, pressure_event;

bool mpu_signals();
void mpu_setup();
bool bmp_signals();
void bmp_setup();
void motor_signals();
void motor_setup();

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  mpu_setup();
  bmp_setup();
  // motor_setup();
}

void loop() {
  if (mpu_signals()) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(RateRoll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(RatePitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(RateYaw);
  }

  if (bmp_signals()) {
    Serial.print("Temperature [*C]= ");
    Serial.print(temp_event.temperature);
    Serial.print(" Pressure [hPa]= ");
    Serial.println(pressure_event.pressure);
  }
  
  // motor_signals();

  delay(500);
}

void mpu_setup() {
  Wire.beginTransmission(I2C_ADDRESS_MPU); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);
}

bool mpu_signals() {
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

void bmp_setup() {
  unsigned status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();
}

bool bmp_signals() {
  if (
    bmp_temp->getEvent(&temp_event) &&
    bmp_pressure->getEvent(&pressure_event)
    )
    return true;
  return false;
}

void motor_setup() {
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

void motor_signals() {
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