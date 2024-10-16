// 192.168.42.1:4242 to access

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
constexpr pin_size_t PIN_GREEN_LED = 22;
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
String client_request;
bool emergency = false;
bool motors_on = false;
int motor_value = 1500;
struct Motor {
  unsigned one = MOTOR_MIN;
  unsigned two = MOTOR_MIN;
  unsigned three = MOTOR_MIN;
  unsigned four = MOTOR_MIN;
} kMotor;
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
};
Rpy<float> rate, cal, desired, error, input, error_prev, i_term_prev;
constexpr Rpy<Pid> kPid = {
  {0.6, 3.5, 0.03},
  {0.6, 3.5, 0.03},
  {2, 12, 0},
};
float rate_roll, rate_pitch, rate_yaw = 0.0;
float rate_roll_cal, rate_pitch_cal, rate_yaw_cal = 0.0;
struct PidOut {
  float value = 0.0;
  float error = 0.0;
  float i_term = 0.0;
};
struct RxOut {
  Rpy<float> rpy;
  float throttle = 0.0;
} rx_out;

bool mpu_signals();
void mpu_setup();
bool bmp_signals();
void bmp_setup();
bool pmon_signals();
void pmon_setup();
bool wifi_signals();
void wifi_setup();
void wifi_state_emergency();
RxOut read_receiver();
void motor_signals();
void motor_setup();
void motor_off();
Rpy<float> angular_rate_of_input(Rpy<float> input);
PidOut pid_equation(float err, Pid pid, float prev_err, float prev_I);
void pid_reset();

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
  mpu_setup();
  bmp_setup();
  motor_setup();

  pinMode(PIN_GREEN_LED, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  const int PRINT_PERIOD_MS = 5000;
  static int print_hold = 0;
  static bool do_print = false;

  if (emergency) {
    motor_off();
    wifi_state_emergency();
    digitalWrite(PIN_LED, LOW);
  }
  else {
    motor_signals();
    digitalWrite(PIN_LED, HIGH);
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

  if (mpu_signals() && do_print) {
    Serial.print("Roll rate [°/s]= ");
    Serial.print(rate_roll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(rate_pitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(rate_yaw);
  }

  if (bmp_signals() && do_print) {
    Serial.print("Temperature [*C]= ");
    Serial.print(temp_event.temperature);
    Serial.print(" Pressure [hPa]= ");
    Serial.println(pressure_event.pressure);
  }

  if (wifi_signals()) {  //&& do_print) {
    Serial.print("Client request: ");
    Serial.println(client_request);
    rx_out = read_receiver();
    desired = angular_rate_of_input(rx_out.rpy);
    error.roll = desired.roll - rate.roll;
    error.pitch = desired.pitch - rate.pitch;
    error.yaw = desired.yaw - rate.yaw;
    PidOut input_roll = pid_equation(error.roll, kPid.roll, error_prev.roll, i_term_prev.roll);
    PidOut input_pitch = pid_equation(error.pitch, kPid.pitch, error_prev.pitch, i_term_prev.pitch);
    PidOut input_yaw = pid_equation(error.yaw, kPid.yaw, error_prev.yaw, i_term_prev.yaw);
    const float MAGIC_MOTOR = 1.024;
    kMotor.one = MAGIC_MOTOR * (rx_out.throttle - input_roll.value - input_pitch.value - input_yaw.value);
    kMotor.two = MAGIC_MOTOR * (rx_out.throttle - input_roll.value + input_pitch.value + input_yaw.value);
    kMotor.three = MAGIC_MOTOR * (rx_out.throttle + input_roll.value + input_pitch.value - input_yaw.value);
    kMotor.four = MAGIC_MOTOR * (rx_out.throttle + input_roll.value - input_pitch.value + input_yaw.value);
    kMotor.one = kMotor.one >= MOTOR_MAX ? MOTOR_MAX : kMotor.one;
    kMotor.two = kMotor.two >= MOTOR_MAX ? MOTOR_MAX : kMotor.two;
    kMotor.three = kMotor.three >= MOTOR_MAX ? MOTOR_MAX : kMotor.three;
    kMotor.four = kMotor.four >= MOTOR_MAX ? MOTOR_MAX : kMotor.four;
    kMotor.one = kMotor.one <= THROTTLE_IDLE ? THROTTLE_IDLE : kMotor.one;
    kMotor.two = kMotor.two <= THROTTLE_IDLE ? THROTTLE_IDLE : kMotor.two;
    kMotor.three = kMotor.three <= THROTTLE_IDLE ? THROTTLE_IDLE : kMotor.three;
    kMotor.four = kMotor.four <= THROTTLE_IDLE ? THROTTLE_IDLE : kMotor.four;
    if (rx_out.throttle < THROTTLE_MIN + 50) {
      kMotor.one = THROTTLE_MIN;
      kMotor.two = THROTTLE_MIN;
      kMotor.three = THROTTLE_MIN;
      kMotor.four = THROTTLE_MIN;
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

void mpu_setup() {
  Wire.beginTransmission(I2C_ADDRESS_MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);

  const unsigned N = 2000;
  float r, p, y = 0.0;
  for (unsigned i = 0; i < N; ++i) {
    if (!mpu_signals())
        continue;
    r += rate_roll;
    p += rate_pitch;
    y += rate_yaw;
    delay(1);
  }
  rate_roll_cal = r / static_cast<float>(N);
  rate_pitch_cal = p / static_cast<float>(N);
  rate_yaw_cal = y / static_cast<float>(N);
  Serial.printf("IMU Calibration. Roll: %.2f Pitch: %.2f Yaw: %.2f\n", rate_roll_cal, rate_pitch_cal, rate_yaw_cal);
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
  Wire.requestFrom(I2C_ADDRESS_MPU, 6);
  if (Wire.available() == 6) {
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();

    rate_roll = (static_cast<float>(GyroX) / DEG_TO_LSB) - rate_roll_cal;
    rate_pitch = (static_cast<float>(GyroY) / DEG_TO_LSB) - rate_pitch_cal;
    rate_yaw = (static_cast<float>(GyroZ) / DEG_TO_LSB) - rate_yaw_cal;

    return true;
  }
  return false;
}

void bmp_setup() {
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

bool bmp_signals() {
  if (
    bmp_temp->getEvent(&temp_event) && bmp_pressure->getEvent(&pressure_event))
    return true;
  return false;
}

void motor_off() {
  m1_esc.writeMicroseconds(0);
  m2_esc.writeMicroseconds(0);
  m3_esc.writeMicroseconds(0);
  m4_esc.writeMicroseconds(0);
}

void motor_setup() {
  m1_esc.attach(PIN_M1);
  m2_esc.attach(PIN_M2);
  m3_esc.attach(PIN_M3);
  m4_esc.attach(PIN_M4);
  motor_off();
  delay(3000);
}

void motor_signals() {
  motor_value = min(max(motor_value, MOTOR_MIN), MOTOR_MAX);

  if (motors_on) {
    digitalWrite(PIN_GREEN_LED, HIGH);
    m1_esc.writeMicroseconds(motor_value);
    m2_esc.writeMicroseconds(motor_value);
    m3_esc.writeMicroseconds(motor_value);
    m4_esc.writeMicroseconds(motor_value);
  }
  else {
    digitalWrite(PIN_GREEN_LED, LOW);
    motor_off();
  }
}

Rpy<float> angular_rate_of_input(Rpy<float> input) {
  const float SLOPE = 0.15;
  const unsigned DEFAULT_INPUT = 1500;
  Rpy<float> desired;
  desired.pitch = SLOPE * (input.pitch - DEFAULT_INPUT);
  desired.roll = SLOPE * (input.roll - DEFAULT_INPUT);
  desired.yaw = SLOPE * (input.yaw - DEFAULT_INPUT);
  return desired;
}

PidOut pid_equation(float err, Pid pid, float prev_err, float prev_i) {
  const int MAGIC_CAP = 400;
  const float MAGIC_TERM = 0.004;
  const unsigned MAGIC_I_DIV = 2;
  float p_err = pid.p * err;
  float i_err = prev_i + pid.i * (err + prev_err) * MAGIC_TERM / MAGIC_I_DIV;
  if (i_err > MAGIC_CAP)
    i_err = MAGIC_CAP;
  else if (i_err < -MAGIC_CAP)
    i_err = -MAGIC_CAP;
  float d_err = pid.d * (err - prev_err ) / MAGIC_TERM;
  float pid_output = p_err + i_err + d_err;
  if (pid_output > MAGIC_CAP)
    pid_output = MAGIC_CAP;
  else if (pid_output < -MAGIC_CAP)
    pid_output = -MAGIC_CAP;
  return {pid_output, err, -pid.i};
}

void pid_reset() {
  error_prev.roll = 0.0;
  error_prev.pitch = 0.0;
  error_prev.yaw = 0.0;
  i_term_prev.roll = 0.0;
  i_term_prev.pitch = 0.0;
  i_term_prev.yaw = 0.0;
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

bool wifi_signals() {
  if ((millis() - last_contact) > EMERGENCY_KILL_MS) {
    emergency = true;
  }

  WiFiClient client = server.accept();
  if (!client)
    return false;

  client_request = client.readStringUntil('\n');
  if (client_request.indexOf("MOTOR%20ON") > 0) {
    Serial.println("Turning motors on");
    motors_on = true;
  }
  else if (client_request.indexOf("MOTOR%20OFF") > 0) {
    Serial.println("Turning motors off");
    motors_on = false;
  }
  else if (client_request.indexOf("MOTOR%20VALUE?motor_value=") > 0) {
    int start_index = client_request.indexOf('=') + 1;
    int end_index = client_request.indexOf(' ', start_index);
    String motor_value_s = client_request.substring(start_index, end_index);
    motor_value = motor_value_s.toInt();
    Serial.printf("Setting motor value to %d\n", motor_value);
  }
  else if (client_request.indexOf("favicon") > 0) {}
  else
    Serial.printf("Failed to parse request %s", client_request);

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

RxOut read_receiver() {
  return {{1500.0, 1500.0, 1500.0}, 1500.0};
}
