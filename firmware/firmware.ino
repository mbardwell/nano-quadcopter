/**
 * @file firmware.ino
 * @brief Firmware for a nano quadcopter project using various sensors and WiFi communication.
 * 
 * This firmware controls a nano quadcopter, handling motor signals, sensor data acquisition,
 * and WiFi communication for remote control and telemetry. It includes PID control for 
 * stabilizing the quadcopter based on IMU and pressure sensor data.
 * 
 * When turned on, the quadcopter will create a WiFi network with the name in WIFI_SSID below.
 * Join that, then open a browser and go to 192.168.42.1:4242 to access the control interface.
 * 
 * @details
 * The firmware includes the following functionalities:
 * - IMU setup and calibration
 * - Pressure sensor setup and altitude calculation
 * - Power monitoring setup and signal processing
 * - WiFi setup and signal processing
 * - Motor control and emergency handling
 * - PID control for roll, pitch, and yaw stabilization
 * 
 * The main loop handles sensor data acquisition, user input processing, PID calculations,
 * and motor signal updates. It also manages emergency states and logs telemetry data.
 * 
 * @note
 * - Ensure the correct wiring of sensors and motors.
 * - Adjust PID coefficients and other constants as needed for your specific quadcopter.
 * - The WiFi SSID and port are hardcoded and should be updated as necessary.
 * 
 * @section Dependencies
 * - Adafruit BMP280 library
 * - CircularBuffer library
 * - Servo library
 * - WiFi library
 * - Wire library
 * 
 * @section Pin Configuration
 * - Motor pins: PIN_M1, PIN_M2, PIN_M3, PIN_M4
 * - I2C pins: PIN_I2C_SDA, PIN_I2C_SCL
 * - LED pins: PIN_LED_MOTOR, PIN_LED_EMERGENCY
 * - Power monitoring pins: PIN_IMON, PIN_VMON
 * 
 * @section Constants
 * - WiFi SSID: WIFI_SSID
 * - WiFi port: WIFI_PORT
 * - Emergency kill timeout: EMERGENCY_KILL_MS
 * - Throttle and ESC limits: THROTTLE_MAX, THROTTLE_IDLE, THROTTLE_MIN, ESC_MAX, ESC_MIN
 * - PID coefficients: kPidCoeffs
 * 
 * @section Structures
 * - Motor: Stores motor signal values.
 * - Pid: Stores PID coefficients.
 * - Rpy: Template structure for roll, pitch, and yaw values.
 * - UserInput: Stores user input values for roll, pitch, yaw, throttle, and motor state.
 * - BmpData: Stores BMP280 sensor data and calculates altitude.
 * - Telemetry: Stores telemetry data for logging.
 * 
 * @section Functions
 * - setup: Initializes all components and sensors.
 * - loop: Main loop handling sensor data, user input, PID control, and motor signals.
 * - imu_setup: Initializes the IMU.
 * - imu_calibration: Calibrates the IMU.
 * - imu_signals: Reads IMU signals.
 * - pressure_setup: Initializes the pressure sensor.
 * - altitude_calibration: Calibrates the altitude.
 * - pressure_signals: Reads pressure sensor signals.
 * - pmon_setup: Initializes power monitoring.
 * - pmon_signals: Reads power monitoring signals.
 * - wifi_setup: Initializes WiFi.
 * - wifi_signals: Processes WiFi signals and user input.
 * - wifi_state_emergency: Handles WiFi signals during emergency state.
 * - motor_setup: Initializes motors.
 * - motor_signals: Updates motor signals.
 * - motor_off: Turns off motors.
 * - angular_rate_of_input: Calculates angular rate from user input.
 * - calculate_motor_signals: Calculates motor signals based on throttle and PID output.
 * - pid_equation: Calculates PID output.
 * - pid_reset: Resets PID memory.
 */
// 

#include <Adafruit_BMP280.h>
#include <CircularBuffer.hpp>
#include <cmath>
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
constexpr unsigned THROTTLE_MAX = 1800;  // Throttle max must leave headroom for roll, pitch, yaw-ing at max throttle
constexpr unsigned THROTTLE_IDLE = 1180;
constexpr unsigned THROTTLE_MIN = 1000;
constexpr float RPY_MAX = 2000.0;
constexpr float RPY_DEFAULT = 1500.0;
constexpr float RPY_MIN = 1000.0;
constexpr unsigned ESC_MAX = 2000;
constexpr unsigned ESC_MIN = 1000;

Servo m1_esc, m2_esc, m3_esc, m4_esc;
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
float current, voltage;
sensors_event_t temp_event, pressure_event;
WiFiServer server(WIFI_PORT);
struct Motor {
  unsigned one = ESC_MIN;
  unsigned two = ESC_MIN;
  unsigned three = ESC_MIN;
  unsigned four = ESC_MIN;
};
int last_contact = 0;
struct Pid {
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
};
/**
 * @brief A template structure representing Roll, Pitch, and Yaw (RPY) values.
 * 
 * Roll, pitch, and yaw directions are defined as follows:
 * - Roll: +ve is clockwise (CW) rotation around the X-axis.
 * - Pitch: +ve is clockwise (CW) rotation around the Y-axis.
 * - Yaw: +ve is clockwise (CW) rotation around the Z-axis.
 * 
 * @tparam T The data type of the roll, pitch, and yaw values (e.g., float, double).
 */
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
struct UserInput {
  Rpy<float> rpy, rpy_coeffs;
  Rpy<bool> rpy_on;
  float throttle;
  bool on;

  void print() const {
    Serial.printf("User Input: Roll=%.2f, Pitch=%.2f, Yaw=%.2f, Throttle=%.2f, Roll On=%s, Pitch On=%s, Yaw On=%s, Motor On=%s\n",
                  rpy.roll, rpy.pitch, rpy.yaw, throttle, rpy_on.roll ? "true" : "false", rpy_on.pitch ? "true" : "false", rpy_on.yaw ? "true" : "false", on ? "true" : "false");
  }
};
struct BmpData {
  float temp;
  float pressure;
  float altitude;
  float altitude_cal;
  bool altitude_cal_set;

  BmpData() : temp(0.0), pressure(0.0), altitude(0.0), altitude_cal(0.0), altitude_cal_set(false) {}

  float calculate_altitude() {
    // Baseline pressure and temperature changes throughout the day
    // This value needs to be paired with a calibration value to be accurate
    constexpr float ATM_PRESSURE_BASELINE = 1013.25;
    altitude = 44330 * (1 - (pow(pressure / ATM_PRESSURE_BASELINE, 1/5.255))) - altitude_cal;
    return altitude;
  }

  void print() const {
    Serial.printf("Temperature: %.2f, Pressure: %.2f, Altitude: %.2f\n", temp, pressure, altitude);
  }
};

struct Telemetry {
  unsigned long time;
  float throttle;
  Rpy<float> input, desired, raw, error, pid;
  Motor motor;
  float voltage;
  float current;
};
CircularBuffer<Telemetry, 1800> telemetry_store;

void imu_setup();
bool imu_calibration(Rpy<float> &);
bool imu_signals(const Rpy<float> &, Rpy<float> &);
void pressure_setup();
bool altitude_calibration(float &);
bool pressure_signals(BmpData &);
void pmon_setup();
bool pmon_signals();
void wifi_setup();
bool wifi_signals(UserInput &, bool &, const WebInterfaceData &);
void wifi_state_emergency(bool &, const WebInterfaceData &);
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
  const float ANGULAR_RATE_EMERGENCY_CUTOFF = 150.0;
  static int print_hold = 0;
  static bool loop_print = false;
  static Rpy<float> imu_cal, imu_rate, pid_mem_err, pid_mem_iterm;
  static Motor m_values;
  static bool emergency = false;
  static BmpData bmp_data;
  static WebInterfaceData web_data;
  static Rpy<Pid> pid_coeffs = {
    {0.6, 3.5, 0.03},
    {0.3, 3.5, 0.03},
    {1.0, 12, 0},
  };
  static UserInput user_input = {Rpy<float>(RPY_DEFAULT, RPY_DEFAULT, RPY_DEFAULT), Rpy<float>{pid_coeffs.roll.p, pid_coeffs.pitch.p, pid_coeffs.yaw.p}, Rpy<bool>{true, true, true}, THROTTLE_IDLE, false};

  // Keep this at the top
  if (emergency) {
    user_input.on = false;
    motor_off();
    wifi_state_emergency(emergency, web_data);
    digitalWrite(PIN_LED_EMERGENCY, HIGH);
    return;
  }
  digitalWrite(PIN_LED_EMERGENCY, LOW);

  if (imu_cal == Rpy<float>()) {
    web_data.imu_cal = imu_calibration(imu_cal);
  }

  if (!bmp_data.altitude_cal_set) {
    web_data.alt_cal = altitude_calibration(bmp_data.altitude_cal);
    bmp_data.altitude_cal_set = true;
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
    web_data.V = voltage;
    web_data.I = current;
  }

  if (imu_signals(imu_cal, imu_rate) && loop_print) {
    Serial.printf("Roll rate [°/s]=%.2f, Pitch rate [°/s]=%.2f, Yaw rate [°/s]=%.2f\n", imu_rate.roll, imu_rate.pitch, imu_rate.yaw);
  }
  if (abs(imu_rate.roll) > ANGULAR_RATE_EMERGENCY_CUTOFF || abs(imu_rate.pitch) > ANGULAR_RATE_EMERGENCY_CUTOFF) {
    Serial.println("Emergency cutoff due to high angular rate");
    emergency = true;
    return;
  }

  if (pressure_signals(bmp_data) && loop_print) {
    bmp_data.print();
    web_data.alt = bmp_data.altitude;
  }

  if (wifi_signals(user_input, emergency, web_data)) {
    user_input.print();
    if (user_input.throttle < THROTTLE_MIN + 50) {
      pid_reset(pid_mem_err, pid_mem_iterm);
    }
  }

  user_input.throttle = min(max(user_input.throttle, THROTTLE_MIN), THROTTLE_MAX);
  Rpy<float> desired = angular_rate_of_input(user_input.rpy);
  Rpy<float> error = desired - imu_rate;
  Rpy<float> pid_out;
  web_data.roll_coeff = pid_coeffs.roll.p;
  web_data.pitch_coeff = pid_coeffs.pitch.p;
  web_data.yaw_coeff = pid_coeffs.yaw.p;
  pid_coeffs.roll.p = user_input.rpy_coeffs.roll;
  pid_coeffs.pitch.p = user_input.rpy_coeffs.pitch;
  pid_coeffs.yaw.p = user_input.rpy_coeffs.yaw;
  pid_out.roll = user_input.rpy_on.roll ? pid_equation(pid_coeffs.roll, error.roll, pid_mem_err.roll, pid_mem_iterm.roll) : 0;
  pid_out.pitch = user_input.rpy_on.pitch ? pid_equation(pid_coeffs.pitch, error.pitch, pid_mem_err.pitch, pid_mem_iterm.pitch) : 0;
  pid_out.yaw = user_input.rpy_on.yaw ? pid_equation(pid_coeffs.yaw, error.yaw, pid_mem_err.yaw, pid_mem_iterm.yaw) : 0;
  if (loop_print) {
    Serial.printf("PID Output: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", pid_out.roll, pid_out.pitch, pid_out.yaw);
  }
  static unsigned first_five = 5;
  if (user_input.on) {
    if (first_five > 0) {
      const unsigned PULSE_LOW = ESC_MIN + 50;
      motor_signals({PULSE_LOW, PULSE_LOW, PULSE_LOW, PULSE_LOW});
      first_five--;
    }
    else {
      m_values = calculate_motor_signals(user_input.throttle, pid_out);
      motor_signals(m_values);
    }
  }
  else {
    motor_off();
    first_five = 5;
  }

  if (!emergency) {
    telemetry_store.push(
    {millis(), 
    user_input.throttle, 
    user_input.rpy,
    desired,
    imu_rate,
    error,
    pid_out,
    m_values,
    voltage,
    current}
    );
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

bool altitude_calibration(float &alt_cal) {
  const unsigned N = 2000;
  unsigned n_fail = 0;
  BmpData once;
  float sum = 0.0;
  for (unsigned i = 0; i < N; ++i) {
    if (!pressure_signals(once)) {
      if (++n_fail > 10)
        return false;
      continue;
    }
    sum = once.calculate_altitude() + sum;
    delay(1);
  }
  alt_cal = sum / static_cast<float>(N);
  Serial.printf("Altitude Calibration: %.2f\n", alt_cal);
  return true;
}

bool pressure_signals(BmpData &data) {
  if (bmp_temp->getEvent(&temp_event) && bmp_pressure->getEvent(&pressure_event)) {
    data.temp = temp_event.temperature;
    data.pressure = pressure_event.pressure;
    static_cast<void>(data.calculate_altitude());
    return true;
  }
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
    return std::min(std::max(val, ESC_MIN), ESC_MAX);
  };

  m1_esc.writeMicroseconds(clamp_motor_values(value.one));
  m2_esc.writeMicroseconds(clamp_motor_values(value.two));
  m3_esc.writeMicroseconds(clamp_motor_values(value.three));
  m4_esc.writeMicroseconds(clamp_motor_values(value.four));
  digitalWrite(PIN_LED_MOTOR, HIGH);
  static int anti_spam = 0;
  if ((millis() - anti_spam) > 100) {
    Serial.printf("Setting motors to %d %d %d %d\n", clamp_motor_values(value.one), clamp_motor_values(value.two), clamp_motor_values(value.three), clamp_motor_values(value.four));
    anti_spam = millis();
  }
}

void motor_off() {
  m1_esc.writeMicroseconds(ESC_MIN);
  m2_esc.writeMicroseconds(ESC_MIN);
  m3_esc.writeMicroseconds(ESC_MIN);
  m4_esc.writeMicroseconds(ESC_MIN);
  digitalWrite(PIN_LED_MOTOR, LOW);
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
    const float SLOPE = 0.15;  // This correspond to 75°/s at max input
    const float DEFAULT_INPUT = 1500.0;
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
    m.one = ESC_MIN;
    m.two = ESC_MIN;
    m.three = ESC_MIN;
    m.four = ESC_MIN;
    return m;
  }

  const float MOTOR_WEAK_COMPENSATION = 1.0;
  m.one = MOTOR_WEAK_COMPENSATION * (throttle + pid_output.roll - pid_output.pitch - pid_output.yaw);
  m.two = (throttle - pid_output.roll - pid_output.pitch + pid_output.yaw);
  m.three = (throttle - pid_output.roll + pid_output.pitch - pid_output.yaw);
  m.four = MOTOR_WEAK_COMPENSATION * (throttle + pid_output.roll + pid_output.pitch + pid_output.yaw);

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

bool wifi_signals(UserInput &user_input, bool &emergency, const WebInterfaceData &web_data) {
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
  else if (client_request.indexOf("roll_toggle") > 0) {
    Serial.println("Toggling roll correction");
    user_input.rpy_on.roll = !user_input.rpy_on.roll;
  }
  else if (client_request.indexOf("pitch_toggle") > 0) {
    Serial.println("Toggling pitch correction");
    user_input.rpy_on.pitch = !user_input.rpy_on.pitch;
  }
  else if (client_request.indexOf("yaw_toggle") > 0) {
    Serial.println("Toggling yaw correction");
    user_input.rpy_on.yaw = !user_input.rpy_on.yaw;
  }
  else if (client_request.indexOf("p_roll?p_roll=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String v = client_request.substring(start_index, end_index);
    user_input.rpy_coeffs.roll = v.toFloat();
    Serial.printf("Changing roll P coefficient to %.2f\n", v.toFloat());
  }
  else if (client_request.indexOf("p_pitch?p_pitch=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String v = client_request.substring(start_index, end_index);
    user_input.rpy_coeffs.pitch = v.toFloat();
    Serial.printf("Changing pitch P coefficient to %.2f\n", v.toFloat());
  }
  else if (client_request.indexOf("p_yaw?p_yaw=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String v = client_request.substring(start_index, end_index);
    user_input.rpy_coeffs.yaw = v.toFloat();
    Serial.printf("Changing yaw P coefficient to %.2f\n", v.toFloat());
  }
  else if (client_request.indexOf("dummyRPY") > 0) {
    user_input.rpy = Rpy<float>(RPY_DEFAULT, RPY_DEFAULT, RPY_DEFAULT);
  }
  else if (client_request.indexOf("download") > 0) {
    Serial.println("Download data");
    client.println("HTTP/1.1 200 OK\r\n");
    client.println("time,throttle,input_roll,input_pitch,input_yaw,desired_roll,desired_pitch,desired_yaw,raw_roll,raw_pitch,raw_yaw,error_roll,error_pitch,error_yaw,pid_roll,pid_pitch,pid_yaw,motor_one,motor_two,motor_three,motor_four,voltage,current");
    for (size_t i = 0; i < telemetry_store.size(); ++i) {
      auto entry = telemetry_store[i];
      client.printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.2f\n",
                    entry.time, entry.throttle, 
                    entry.input.roll, entry.input.pitch, entry.input.yaw,
                    entry.desired.roll, entry.desired.pitch, entry.desired.yaw, 
                    entry.raw.roll, entry.raw.pitch, entry.raw.yaw,
                    entry.error.roll, entry.error.pitch, entry.error.yaw, 
                    entry.pid.roll, entry.pid.pitch, entry.pid.yaw, 
                    entry.motor.one, entry.motor.two, entry.motor.three, entry.motor.four, 
                    entry.voltage, entry.current
                    );
    }
    client.flush();
    last_contact = millis();
    return true;
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else
    Serial.printf("Failed to parse request %s\n", client_request.c_str());
  
  client.print(html_header);
  client.print(html_running(web_data));
  client.flush();

  last_contact = millis();
  return true;
}

void wifi_state_emergency(bool &emergency, const WebInterfaceData &web_data) {
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
    client.print(html_running(web_data));
    client.flush();
    return;
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else
    Serial.printf("Failed to parse request %s\n", client_request.c_str());

  client.print(html_header);
  client.print(html_emergency);
  client.flush();

  last_contact = millis();
}
