// 192.168.42.1:4242 to access
// TODO: Handle anything static_cast<void>'d
// TODO: Clear up any MAGIC numbers

#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <WiFi.h>
#include <Wire.h>

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
constexpr unsigned MOTOR_MAX = 1999; // Translates to 100% motor power. Max is theoretically 2000, but this works
constexpr unsigned MOTOR_MIN = THROTTLE_IDLE; // Translates to 0% motor power. Min is theoretically 1000, but this works
const String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
const String html_hd = "<!DOCTYPE html><html><head><title>Quadcopter Control</title></head><body><div id='main'><h1>Quadcopter Control</h1>";
auto html_IV = [](float I, float V) -> String { return "<body><h2>Current and Voltage Information</h2><div class='info'><p><strong>Current:</strong> " + String(I) + " A</p><p><strong>Voltage:</strong> " + String(V) + " V</p></div></body></html>"; };
const String html_motor_on = "<form id='F1' action='MOTOR ON'><input type='submit' value='MOTOR ON' style='padding: 120px 140px; font-size: 124px; margin: 110px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 15px;'></form><br>";
const String html_motor_off = "<form id='F2' action='MOTOR OFF'><input type='submit' value='MOTOR OFF' style='padding: 120px 140px; font-size: 124px; margin: 110px; cursor: pointer; background-color: #f44336; color: white; border: none; border-radius: 15px;'></form><br>";
const String html_motor_val = "<form id='F3' action='MOTOR VALUE'><input type='number' name='motor_value' required style='padding: 20px; font-size: 24px; margin: 10px; width: 100%; max-width: 300px;'></form>";
const String html_set_mval = "<input type='submit' value='Set Motor Value' style='padding: 20px 40px; font-size: 24px; margin: 10px; cursor: pointer; background-color: #2196F3; color: white; border: none; border-radius: 5px;'></form></div></body></html>";
const String html_ft = "</div></body></html>";
const String html_emergency_1 = "<!DOCTYPE html><html><head><title>Emergency State</title></head><body><div id='main'><h1>Emergency State</h1>";
const String html_emergency_2 = "<form id='F1' action='RESET'><input type='submit' value='RESET' style='padding: 120px 140px; font-size: 124px; margin: 110px; cursor: pointer; background-color: #4CAF50; color: white; border: none; border-radius: 15px;'></form><br>";


Servo m1_esc, m2_esc, m3_esc, m4_esc;
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float current, voltage;
sensors_event_t temp_event, pressure_event;
WiFiServer server(WIFI_PORT);
bool emergency = false;
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
  T roll = T();
  T pitch = T();
  T yaw = T();

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
constexpr Rpy<Pid> kPidCoeffs = {
  {0.6, 3.5, 0.03},
  {0.6, 3.5, 0.03},
  {2, 12, 0},
};
struct UserInput {
  Rpy<float> rpy;
  float throttle;
  bool on;
};

void imu_setup();
bool imu_calibration(Rpy<float> &);
bool imu_signals(const Rpy<float> &, Rpy<float> &);
void pressure_setup();
bool pressure_signals();
void pmon_setup();
bool pmon_signals();
void wifi_setup();
bool wifi_signals(UserInput &);
void wifi_state_emergency();
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
  static bool do_print = false;
  static Rpy<float> imu_cal, imu_rate, pid_mem_err, pid_mem_iterm;
  static Motor m_values;
  UserInput user_input;

  if (emergency) {
    motor_off();
    wifi_state_emergency();
    digitalWrite(PIN_LED_EMERGENCY, HIGH);
    return;
  }
  digitalWrite(PIN_LED_EMERGENCY, LOW);

  if (imu_cal == Rpy<float>()) {
    static_cast<void>(imu_calibration(imu_cal));
  }

  if ((millis() - print_hold) > PRINT_PERIOD_MS) {
    do_print = true;
    print_hold = millis();
  }

  if (pmon_signals() && do_print) {
    Serial.print("Voltage [V]= ");
    Serial.print(voltage);
    Serial.print(" Current [A]= ");
    Serial.println(current);
  }

  if (imu_signals(imu_cal, imu_rate) && do_print) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(imu_rate.roll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(imu_rate.pitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(imu_rate.yaw);
  }

  if (pressure_signals() && do_print) {
    Serial.print("Temperature [*C]= ");
    Serial.print(temp_event.temperature);
    Serial.print(" Pressure [hPa]= ");
    Serial.println(pressure_event.pressure);
  }

  if (wifi_signals(user_input)) {
    if (user_input.throttle < THROTTLE_MIN + 50) {
      pid_reset(pid_mem_err, pid_mem_iterm);
      motor_off();
    }
    else {
      Rpy<float> desired = angular_rate_of_input(user_input.rpy);
      Rpy<float> error = desired - imu_rate;
      Rpy<float> pid_out;
      pid_out.roll = pid_equation(kPidCoeffs.roll, error.roll, pid_mem_err.roll, pid_mem_iterm.roll);
      pid_out.pitch = pid_equation(kPidCoeffs.pitch, error.pitch, pid_mem_err.pitch, pid_mem_iterm.pitch);
      pid_out.yaw = pid_equation(kPidCoeffs.yaw, error.yaw, pid_mem_err.yaw, pid_mem_iterm.yaw);
      m_values = calculate_motor_signals(user_input.throttle, pid_out);
    }
  }

  do_print = false;
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
  Rpy<float> r_sum, r;
  for (unsigned i = 0; i < N; ++i) {
    if (!imu_signals(Rpy<float>(), r)) {
      if (++n_fail > 10)
        return false;
      continue;
    }
    r_sum = r + r_sum;
    delay(1);
  }
  cal.roll = r.roll / static_cast<float>(N);
  cal.pitch = r.pitch / static_cast<float>(N);
  cal.yaw = r.yaw / static_cast<float>(N);
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
  if (
    bmp_temp->getEvent(&temp_event) && bmp_pressure->getEvent(&pressure_event))
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
  m1_esc.writeMicroseconds(value.one);
  m2_esc.writeMicroseconds(value.two);
  m3_esc.writeMicroseconds(value.three);
  m4_esc.writeMicroseconds(value.four);
  digitalWrite(PIN_LED_MOTOR, HIGH);
}

void motor_off() {
  // TODO: Use MOTOR_MIN?
  m1_esc.writeMicroseconds(0);
  m2_esc.writeMicroseconds(0);
  m3_esc.writeMicroseconds(0);
  m4_esc.writeMicroseconds(0);
  digitalWrite(PIN_LED_MOTOR, HIGH);
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

  auto clamp_motor_values = [](unsigned &val) {
    val = std::min(std::max(val, MOTOR_MIN), MOTOR_MAX);
  };
  clamp_motor_values(m.one);
  clamp_motor_values(m.two);
  clamp_motor_values(m.three);
  clamp_motor_values(m.four);

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
  float pid_output = p_err + i_err + d_err;
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

bool wifi_signals(UserInput &user_input) {
  String client_request;

  if ((millis() - last_contact) > EMERGENCY_KILL_MS) {
    emergency = true;
  }

  WiFiClient client = server.accept();
  if (!client)
    return false;

  client_request = client.readStringUntil('\n');
  if (client_request.indexOf("MOTOR%20ON") > 0) {
    Serial.println("Turning motors on");
    user_input.on = true;
  }
  else if (client_request.indexOf("MOTOR%20OFF") > 0) {
    Serial.println("Turning motors off");
    user_input.on = false;
  }
  else if (client_request.indexOf("MOTOR%20VALUE?motor_value=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String motor_value_s = client_request.substring(start_index, end_index);
    user_input.throttle = motor_value_s.toFloat();
    Serial.printf("Setting motor value to %d\n", user_input.throttle);
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else
    Serial.printf("Failed to parse request %s", client_request);

  user_input.rpy = Rpy<float>();  // TODO: Get from client_request

  client.print(header);
  client.print(html_hd);
  client.print(html_IV(current, voltage));
  client.print(html_motor_on);
  client.print(html_motor_off);
  client.print(html_motor_val);
  client.print(html_set_mval);
  client.print(html_ft);
  client.flush();

  last_contact = millis();
  return true;
}

void wifi_state_emergency() {
  String client_request;
  WiFiClient client = server.accept();

  if (!client)
    return;

  client_request = client.readStringUntil('\n');
  if (client_request.indexOf("RESET") > 0) {
    Serial.println("Exiting emergency state");
    emergency = false;
    return;
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else
    Serial.printf("Failed to parse request %s", client_request);

  client.print(header);
  client.print(html_emergency_1);
  client.print(html_emergency_2);

  last_contact = millis();
}
