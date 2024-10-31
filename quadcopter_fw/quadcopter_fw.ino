// 192.168.42.1:4242 to access
// TODO: Handle anything static_cast<void>'d
// TODO: Clear up any MAGIC numbers

#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <WiFi.h>
#include <Wire.h>

#include "html.hpp"

constexpr pin_size_t PIN_M1 = 0;
constexpr pin_size_t PIN_M2 = 2;
constexpr pin_size_t PIN_M3 = 4;
constexpr pin_size_t PIN_M4 = 6;
constexpr pin_size_t PIN_I2C_SDA = 8;
constexpr pin_size_t PIN_I2C_SCL = 9;
constexpr pin_size_t PIN_LED_MOTOR = 22;
constexpr pin_size_t PIN_LED_EMERGENCY = PIN_LED;
constexpr pin_size_t PIN_IMON = A0;
constexpr pin_size_t PIN_VMON = A1;
constexpr byte I2C_ADDRESS_MPU = 0x68;
constexpr char WIFI_SSID[] = "FBI-surveillance-van";
constexpr unsigned WIFI_PORT = 4242;
constexpr unsigned EMERGENCY_KILL_MS = 30000;
constexpr unsigned THROTTLE_MAX = 1800;
constexpr unsigned THROTTLE_IDLE = 1180;
constexpr unsigned THROTTLE_MIN = 1000;
constexpr float RPY_MAX = 2000.0;
constexpr float RPY_DEFAULT = 1500.0;
constexpr float RPY_MIN = 1000.0;
constexpr unsigned MOTOR_MAX = 1999; // Translates to 100% motor power. Max is theoretically 2000, but this works
constexpr unsigned MOTOR_MIN = THROTTLE_IDLE; // Translates to 0% motor power. Min is theoretically 1000, but this works

Servo m1_esc, m2_esc, m3_esc, m4_esc;
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float current, voltage;
sensors_event_t temp_event, pressure_event;
WiFiServer server(WIFI_PORT);
struct Motor {
  unsigned one = MOTOR_MIN;
  unsigned two = MOTOR_MIN;
  unsigned three = MOTOR_MIN;
  unsigned four = MOTOR_MIN;
};
int last_contact = 0;
struct Pid {
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
};
template<typename T>
struct Rpy {
  T roll;
  T pitch;
  T yaw;

  Rpy() = default;
  Rpy(T r, T p, T y) : roll(r), pitch(p), yaw(y) {};

  Rpy<T> operator+(const Rpy<T> &rhs) const {
    return {roll + rhs.roll, pitch + rhs.pitch, yaw + rhs.yaw};
  }

  Rpy<T> operator-(const Rpy<T> &rhs) const {
    return {roll - rhs.roll, pitch - rhs.pitch, yaw - rhs.yaw};
  }

  bool operator==(const Rpy<T> &rhs) const {
    return roll == rhs.roll && pitch == rhs.pitch && yaw == rhs.yaw;
  }
};
const Rpy<Pid> kPidCoeffs = {
  {0.6, 3.5, 0.03},
  {0.6, 3.5, 0.03},
  {2, 12, 0},
};
struct UserInput {
  Rpy<float> rpy;
  float throttle;
  bool on;

  void print() const {
    Serial.printf("User Input: Roll=%.2f, Pitch=%.2f, Yaw=%.2f, Throttle=%.2f, On=%s\n", 
                  rpy.roll, rpy.pitch, rpy.yaw, throttle, on ? "true" : "false");
  }
};

void imu_setup();
bool imu_calibration(Rpy<float> &);
bool imu_signals(const Rpy<float> &, Rpy<float> &);
void pressure_setup();
bool pressure_signals();
void pmon_setup();
bool pmon_signals();
void wifi_setup();
bool wifi_signals(UserInput &, bool &);
void wifi_state_emergency(bool &);
void motor_setup();
void motor_signals(const Motor &);
void motor_off();
Rpy<float> angular_rate_of_input(const Rpy<float> &);
float pid_equation(const Pid, float, float &, float &);
void pid_reset(Rpy<float> &, Rpy<float> &);
Motor calculate_motor_signals(float, const Rpy<float> &);

void setup() {
  int MAX_HOLD_MS = 2000;
  Serial.begin(115200);
  while (!Serial && (millis() < MAX_HOLD_MS))
    ;

  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  wifi_setup();
  pmon_setup();
  imu_setup();
  pressure_setup();
  motor_setup();

  pinMode(PIN_LED_MOTOR, OUTPUT);
  pinMode(PIN_LED_EMERGENCY, OUTPUT);
}

void loop() {
  const int PRINT_PERIOD_MS = 5000;
  static int print_hold = 0;
  static bool loop_print = false;
  static Rpy<float> imu_cal, imu_rate, pid_mem_err, pid_mem_iterm;
  static Motor m_values;
  static bool emergency = false;

  // Keep this at the top
  if (emergency) {
    motor_off();
    wifi_state_emergency(emergency);
    digitalWrite(PIN_LED_EMERGENCY, HIGH);
    return;
  }
  digitalWrite(PIN_LED_EMERGENCY, LOW);

  if (imu_cal == Rpy<float>()) {
    if (!imu_calibration(imu_cal))
      Serial.println("IMU calibration failed");
  }

  if ((millis() - print_hold) > PRINT_PERIOD_MS) {
    loop_print = true;
    print_hold = millis();
  }

  if (pmon_signals() && loop_print) {
    Serial.print("Voltage [V]= ");
    Serial.print(voltage);
    Serial.print(" Current [A]= ");
    Serial.println(current);
  }

  if (imu_signals(imu_cal, imu_rate) && loop_print) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(imu_rate.roll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(imu_rate.pitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(imu_rate.yaw);
  }

  if (pressure_signals() && loop_print) {
    Serial.print("Temperature [*C]= ");
    Serial.print(temp_event.temperature);
    Serial.print(" Pressure [hPa]= ");
    Serial.println(pressure_event.pressure);
  }

  static UserInput user_input = {Rpy<float>(RPY_DEFAULT, RPY_DEFAULT, RPY_DEFAULT), THROTTLE_IDLE, false};
  user_input.throttle = 1500;  // Remove me
  if (wifi_signals(user_input, emergency)) {
    user_input.print();
    if (user_input.throttle < THROTTLE_MIN + 50) {
      pid_reset(pid_mem_err, pid_mem_iterm);
    }
  }

  Rpy<float> desired = angular_rate_of_input(user_input.rpy);
  Rpy<float> error = desired - imu_rate;
  Rpy<float> pid_out;
  pid_out.roll = pid_equation(kPidCoeffs.roll, error.roll, pid_mem_err.roll, pid_mem_iterm.roll);
  pid_out.pitch = pid_equation(kPidCoeffs.pitch, error.pitch, pid_mem_err.pitch, pid_mem_iterm.pitch);
  pid_out.yaw = pid_equation(kPidCoeffs.yaw, error.yaw, pid_mem_err.yaw, pid_mem_iterm.yaw);
  if (loop_print) {
    Serial.printf("PID Output: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", pid_out.roll, pid_out.pitch, pid_out.yaw);
  }
  if (user_input.on) {
    m_values = calculate_motor_signals(user_input.throttle, pid_out);
    motor_signals(m_values);
  }
  else {
    motor_off();
  }

  loop_print = false;
}

void pmon_setup() {
  pinMode(PIN_IMON, INPUT);
  pinMode(PIN_VMON, INPUT);
}

bool pmon_signals() {
  const float VMAX = 3.3;
  const float ADC_MAX = 1023.0;
  const int BTS_RATIO = 14000;
  const int LOWER_R1 = 503;
  const int LOWER_R4 = 1000;
  const int LOWER_R5 = 330;
  static int _voltage, _current;
  _current = analogRead(PIN_IMON);
  _voltage = analogRead(PIN_VMON);
  current = _current * (VMAX / ADC_MAX) * BTS_RATIO / LOWER_R1;
  voltage = _voltage * (VMAX / ADC_MAX) * ((LOWER_R4 + LOWER_R5) / (LOWER_R5));
  return true;
}

void imu_setup() {
  Wire.beginTransmission(I2C_ADDRESS_MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);
}

bool imu_calibration(Rpy<float> &cal) {
  const unsigned N = 2000;
  unsigned n_fail = 0;
  Rpy<float> r_sum, r_once;
  for (unsigned i = 0; i < N; ++i) {
    if (!imu_signals(Rpy<float>(), r_once)) {
      if (++n_fail > 10)
        return false;
      continue;
    }
    r_sum = r_once + r_sum;
    delay(1);
  }
  cal.roll = r_sum.roll / static_cast<float>(N);
  cal.pitch = r_sum.pitch / static_cast<float>(N);
  cal.yaw = r_sum.yaw / static_cast<float>(N);
  Serial.printf("IMU Calibration. Roll: %.2f Pitch: %.2f Yaw: %.2f\n", cal.roll, cal.pitch, cal.yaw);
  return true;
}

bool imu_signals(const Rpy<float> &cal, Rpy<float> &gyro) {
  const float DEG_TO_LSB = 65.5;
  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;
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
  Wire.requestFrom(I2C_ADDRESS_MPU, 6);
  if (Wire.available() == 6) {
    raw_x = Wire.read() << 8 | Wire.read();
    raw_y = Wire.read() << 8 | Wire.read();
    raw_z = Wire.read() << 8 | Wire.read();
    gyro.roll = (static_cast<float>(raw_x) / DEG_TO_LSB) - cal.roll;
    gyro.pitch = (static_cast<float>(raw_y) / DEG_TO_LSB) - cal.pitch;
    gyro.yaw = (static_cast<float>(raw_z) / DEG_TO_LSB) - cal.yaw;
    return true;
  }
  return false;
}

void pressure_setup() {
  unsigned status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
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

bool pressure_signals() {
  if (bmp_temp->getEvent(&temp_event) && bmp_pressure->getEvent(&pressure_event))
    return true;
  return false;
}

void motor_setup() {
  m1_esc.attach(PIN_M1);
  m2_esc.attach(PIN_M2);
  m3_esc.attach(PIN_M3);
  m4_esc.attach(PIN_M4);
  motor_off();
  delay(3000);
}

void motor_signals(const Motor &value) {
  auto clamp_motor_values = [](unsigned val) -> unsigned {
    return std::min(std::max(val, MOTOR_MIN), MOTOR_MAX);
  };

  m1_esc.writeMicroseconds(clamp_motor_values(value.one));
  m2_esc.writeMicroseconds(clamp_motor_values(value.two));
  m3_esc.writeMicroseconds(clamp_motor_values(value.three));
  m4_esc.writeMicroseconds(clamp_motor_values(value.four));
  digitalWrite(PIN_LED_MOTOR, HIGH);
  static int anti_spam = 0;
  if ((millis() - anti_spam) > 100) {
    Serial.printf("Setting motors to %d %d %d %d\n", value.one, value.two, value.three, value.four);
    anti_spam = millis();
  }
}

void motor_off() {
  m1_esc.writeMicroseconds(MOTOR_MIN);
  m2_esc.writeMicroseconds(MOTOR_MIN);
  m3_esc.writeMicroseconds(MOTOR_MIN);
  m4_esc.writeMicroseconds(MOTOR_MIN);
  digitalWrite(PIN_LED_MOTOR, HIGH);
  static int anti_spam = 0;
  if ((millis() - anti_spam) > 5000) {
    Serial.println("Motors turned off");
    anti_spam = millis();
  }
}

/**
 * @brief Calculate the angular rate of the input
 * @details The input is a user value from the radio transmitter
 * we need to convert this to an angular rate that the PID controller can use
 * @param input The input from the user
 * @return Rpy<float> The angular rate of the input
 */
Rpy<float> angular_rate_of_input(const Rpy<float> &input) {
  auto calculate_desired = [](float input_value) -> float {
    const float SLOPE = 0.15;
    const unsigned DEFAULT_INPUT = 1500;
    return SLOPE * (input_value - DEFAULT_INPUT);
  };

  Rpy<float> desired;
  desired.roll = calculate_desired(input.roll);
  desired.pitch = calculate_desired(input.pitch);
  desired.yaw = calculate_desired(input.yaw);
  return desired;
}

Motor calculate_motor_signals(float throttle, const Rpy<float> &pid_output) {
  Motor m;
  if (throttle < THROTTLE_MIN + 50) {
    m.one = MOTOR_MIN;
    m.two = MOTOR_MIN;
    m.three = MOTOR_MIN;
    m.four = MOTOR_MIN;
    return m;
  }

  const float MAGIC_MOTOR = 1.024;
  m.one = MAGIC_MOTOR * (throttle - pid_output.roll - pid_output.pitch - pid_output.yaw);
  m.two = MAGIC_MOTOR * (throttle - pid_output.roll + pid_output.pitch + pid_output.yaw);
  m.three = MAGIC_MOTOR * (throttle + pid_output.roll + pid_output.pitch - pid_output.yaw);
  m.four = MAGIC_MOTOR * (throttle + pid_output.roll - pid_output.pitch + pid_output.yaw);

  return m;
}

float pid_equation(const Pid constants, float err, float &prev_err, float &prev_iterm) {
  const int MAGIC_CAP = 400;
  const float MAGIC_TERM = 0.004;
  const unsigned MAGIC_I_DIV = 2;
  float p_err = constants.p * err;
  float i_err = prev_iterm + constants.i * (err + prev_err) * MAGIC_TERM / MAGIC_I_DIV;
  i_err = min(max(i_err, -MAGIC_CAP), MAGIC_CAP);
  float d_err = constants.d * (err - prev_err ) / MAGIC_TERM;
  float pid_output = p_err;// + i_err + d_err;
  pid_output = min(max(pid_output, -MAGIC_CAP), MAGIC_CAP);
  prev_err = err;
  prev_iterm = i_err;
  return pid_output;
}

void pid_reset(Rpy<float> &prev_err, Rpy<float> &prev_iterm) {
  prev_err = Rpy<float>();
  prev_iterm = Rpy<float>();
}

void wifi_setup() {
  WiFi.mode(WIFI_AP);
  Serial.printf("Connecting to '%s'\n", WIFI_SSID);
  WiFi.beginAP(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.printf("\nConnected to WiFi\n\nConnect to server at %s:%d\n", WiFi.localIP().toString().c_str(), WIFI_PORT);
  server.begin();
}

bool wifi_signals(UserInput &user_input, bool &emergency) {
  String client_request;

  if ((millis() - last_contact) > EMERGENCY_KILL_MS) {
    emergency = true;
  }

  WiFiClient client = server.accept();
  if (!client)
    return false;

  client_request = client.readStringUntil('\n');
  Serial.printf("Client request: %s\n", client_request.c_str());
  if (client_request.indexOf("motor_on") > 0) {
    Serial.println("Turning motors on");
    user_input.on = true;
  }
  else if (client_request.indexOf("motor_off") > 0) {
    Serial.println("Turning motors off");
    user_input.on = false;
  }
  else if (client_request.indexOf("motor_value?motor_value=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String motor_value_s = client_request.substring(start_index, end_index);
    user_input.throttle = motor_value_s.toFloat();
    Serial.printf("Setting motor value to %.2f\n", user_input.throttle);
  }
  else if (client_request.indexOf("dummyRPY") > 0) {
    user_input.rpy = Rpy<float>(RPY_DEFAULT, RPY_DEFAULT, RPY_DEFAULT);
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else if (client_request.indexOf("GET /") > 0) {}
  else
    Serial.printf("Failed to parse request %s\n", client_request.c_str());
  
  client.print(html_header);
  client.print(html_running(current, voltage));
  client.flush();

  last_contact = millis();
  return true;
}

void wifi_state_emergency(bool &emergency) {
  String client_request;
  WiFiClient client = server.accept();

  if (!client)
    return;

  client_request = client.readStringUntil('\n');
  Serial.printf("Client request: %s\n", client_request.c_str());
  if (client_request.indexOf("reset") > 0) {
    Serial.println("Exiting emergency state");
    emergency = false;
    client.print(html_header);
    client.print(html_running(current, voltage));
    client.flush();
    return;
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else if (client_request.indexOf("GET /") > 0) {}
  else
    Serial.printf("Failed to parse request %s\n", client_request.c_str());

  client.print(html_header);
  client.print(html_emergency);
  client.flush();

  last_contact = millis();
}
