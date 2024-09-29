#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <Wire.h>

constexpr pin_size_t PIN_M1 = 0;
constexpr pin_size_t PIN_M2 = 2;
constexpr pin_size_t PIN_M3 = 4;
constexpr pin_size_t PIN_M4 = 6;
constexpr pin_size_t PIN_I2C_SDA = 8;
constexpr pin_size_t PIN_I2C_SCL = 9;
constexpr pin_size_t PIN_GREEN_LED = 22;
constexpr pin_size_t PIN_IMON = A0;
constexpr pin_size_t PIN_VMON = A1;
constexpr unsigned MAX_RUNS = 1;
constexpr byte I2C_ADDRESS_MPU = 0x68;

Servo m1_esc, m2_esc, m3_esc, m4_esc;
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float current, voltage;
float rate_roll, rate_pitch, rate_yaw;
sensors_event_t temp_event, pressure_event;

bool mpu_signals();
void mpu_setup();
bool bmp_signals();
void bmp_setup();
void motor_signals();
void motor_setup();
bool pmon_signals();
void pmon_setup();

void setup() {
  Serial.begin(115200);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  pmon_setup();
  mpu_setup();
  bmp_setup();
  // motor_setup();
}

void loop() {
  if (pmon_signals()) {
    Serial.print("Voltage [V]= ");
    Serial.print(voltage);
    Serial.print(" Current [A]= ");
    Serial.println(current);
  }

  if (mpu_signals()) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(rate_roll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(rate_pitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(rate_yaw);
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

void pmon_setup() {
  pinMode(PIN_IMON, INPUT);
  pinMode(PIN_VMON, INPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
}

bool pmon_signals() {
  const float VMAX = 3.3;
  const float ADC_MAX = 1023.0;
  const int BTS_RATIO = 14000;
  const int LOWER_R1 = 503;
  const int LOWER_R4 = 1000;
  const int LOWER_R5 = 330;
  const int RPT_TIME_MS = 1000;
  static int hold = 0;
  static int _voltage, _current;
  _current = analogRead(PIN_IMON);
  _voltage = analogRead(PIN_VMON);
  current = _current * (VMAX / ADC_MAX) * BTS_RATIO / LOWER_R1;
  voltage = _voltage * (VMAX / ADC_MAX) * ((LOWER_R4 + LOWER_R5) / (LOWER_R5));
  if (millis() - hold > RPT_TIME_MS) {
    digitalWrite(PIN_GREEN_LED, digitalRead(PIN_GREEN_LED) == HIGH ? LOW : HIGH);
    hold = millis();
  }
  return true;
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
    
    rate_roll = static_cast<float>(GyroX) / DEG_TO_LSB;
    rate_pitch = static_cast<float>(GyroY) / DEG_TO_LSB;
    rate_yaw = static_cast<float>(GyroZ) / DEG_TO_LSB;
    
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
  m1_esc.attach(PIN_M1);
  m2_esc.attach(PIN_M2);
  m3_esc.attach(PIN_M3);
  m4_esc.attach(PIN_M4);
  Serial.begin(115200);
  m1_esc.writeMicroseconds(0);
  m2_esc.writeMicroseconds(0);
  m3_esc.writeMicroseconds(0);
  m4_esc.writeMicroseconds(0);
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
    m1_esc.writeMicroseconds(0);
    m2_esc.writeMicroseconds(0);
    m3_esc.writeMicroseconds(0);
    m4_esc.writeMicroseconds(0);
    delay(1000);
    return;
  }

  if ((millis() - hold) > AUTO_START_EVERY_MS) {
    for (value = AUTO_MIN_VALUE; value < AUTO_MAX_VALUE; value++) {
      m1_esc.writeMicroseconds(value);
      m4_esc.writeMicroseconds(value);
      m3_esc.writeMicroseconds(value);
      m2_esc.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }
    for (value = AUTO_MAX_VALUE; value > AUTO_MIN_VALUE; value--) {
      Serial.println(value);
      m1_esc.writeMicroseconds(value);
      m2_esc.writeMicroseconds(value);
      m3_esc.writeMicroseconds(value);
      m4_esc.writeMicroseconds(value);
      delay(AUTO_STAGE_SIT_MS);
    }

    m1_esc.writeMicroseconds(0);
    m2_esc.writeMicroseconds(0);
    m3_esc.writeMicroseconds(0);
    m4_esc.writeMicroseconds(0);

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