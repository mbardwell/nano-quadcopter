#pragma once

#include <Arduino.h>

const String html_header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";

struct WebInterfaceData {
    float I;
    float V;
    float alt;
    bool alt_cal;
    bool imu_cal;
    float roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_p, yaw_i, yaw_d;
};

auto html_running = [](const WebInterfaceData &data) -> String {
    return R"(
<!DOCTYPE html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Quadcopter Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background-color: #f0f0f0;
            margin: 0;
            padding: 0;
        }
        #main {
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #fff;
            border-radius: 10px;
            box-shadow: 0px 4px 10px rgba(0, 0, 0, 0.1);
        }
        h1, h2 {
            color: #333;
        }
        .info {
            margin: 20px 0;
        }
        .info p {
            font-size: 18px;
            margin: 10px 0;
        }
        .info span {
            font-weight: bold;
        }
        .current {
            color: #4CAF50;
        }
        .voltage {
            color: #2196F3;
        }
        .altitude {
            color: #FF9800;
        }
        .imu-cal {
            color: #9C27B0;
        }
        .alt-cal {
            color: #E91E63;
        }
        .button {
            padding: 10px 20px;
            font-size: 16px;
            margin: 10px;
            cursor: pointer;
            color: white;
            border: none;
            border-radius: 10px;
        }
        .green-button {
            background-color: #4CAF50;
        }
        .red-button {
            background-color: #f44336;
        }
        .blue-button {
            background-color: #008CBA;
        }
        form {
            margin: 10px 0;
        }
        input[type="number"] {
            padding: 5px;
            font-size: 9px;
            margin: 5px;
            width: 50%;
            max-width: 150px;
        }
    </style>
</head>
<body>
    <div id="main">
        <h1>Quadcopter Control</h1>
        <div class="info">
            <span class="current">Current:</span> )" + String(data.I) + R"( A
            <span class="voltage">Voltage:</span> )" + String(data.V) + R"( V
            <span class="altitude">Altitude:</span> )" + String(data.alt) + R"( m
            <span class="imu-cal">IMU Cal:</span> )" + String(data.imu_cal ? "Yes" : "No") + R"(
            <span class="alt-cal">Alt Cal:</span> )" + String(data.alt_cal ? "Yes" : "No") + R"(
        </div>

        <form id="F1" action="motor_on" method="POST">
            <input type="submit" value="motor_on" class="button green-button">
        </form>

        <form id="F2" action="motor_off" method="POST">
            <input type="submit" value="motor_off" class="button red-button">
        </form>

        <form id="F3" action="motor_value" method="GET">
            <label for="motor_value"><strong>set_motor_value:</strong></label><br>
            <input type="number" name="motor_value" id="motor_value" required>
        </form>
        
        <div class="info">
            <h2>PID Coefficients</h2>
            <table style="margin: 0 auto;">
            <tr>
                <th></th>
                <th>P</th>
                <th>I</th>
                <th>D</th>
            </tr>
            <tr>
                <td>Roll</td>
                <form id="F10" action="p_roll" method="GET"><td>)" + String(data.roll_p) + R"(<input type="number" name="p_roll" id="p_roll" step="any" required></td></form>
                <form id="F11" action="i_roll" method="GET"><td>)" + String(data.roll_i) + R"(<input type="number" name="i_roll" id="i_roll" step="any" required></td></form>
                <form id="F12" action="d_roll" method="GET"><td>)" + String(data.roll_d) + R"(<input type="number" name="d_roll" id="d_roll" step="any" required></td></form>
            </tr>
            <tr>
                <td>Pitch</td>
                <form id="F13" action="p_pitch" method="GET"><td>)" + String(data.pitch_p) + R"(<input type="number" name="p_pitch" id="p_pitch" step="any" required></td></form>
                <form id="F14" action="i_pitch" method="GET"><td>)" + String(data.pitch_i) + R"(<input type="number" name="i_pitch" id="i_pitch" step="any" required></td></form>
                <form id="F15" action="d_pitch" method="GET"><td>)" + String(data.pitch_d) + R"(<input type="number" name="d_pitch" id="d_pitch" step="any" required></td></form>
            </tr>
            <tr>
                <td>Yaw</td>
                <form id="F16" action="p_yaw" method="GET"><td>)" + String(data.yaw_p) + R"(<input type="number" name="p_yaw" id="p_yaw" step="any" required></td></form>
                <form id="F17" action="i_yaw" method="GET"><td>)" + String(data.yaw_i) + R"(<input type="number" name="i_yaw" id="i_yaw" step="any" required></td></form>
                <form id="F18" action="d_yaw" method="GET"><td>)" + String(data.yaw_d) + R"(<input type="number" name="d_yaw" id="d_yaw" step="any" required></td></form>
            </tr>
            </table>
        </div>

        <div style="display: flex; justify-content: center;">
            <form id="F20" action="roll_toggle" method="POST">
            <input type="submit" value="roll_toggle" class="button green-button">
            </form>

            <form id="F21" action="pitch_toggle" method="POST">
            <input type="submit" value="pitch_toggle" class="button green-button">
            </form>

            <form id="F22" action="yaw_toggle" method="POST">
            <input type="submit" value="yaw_toggle" class="button green-button">
            </form>
        </div>
    </div>
</body>
</html>
)";
};

const String html_emergency = R"(
<!DOCTYPE html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Emergency State</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            background-color: #f0f0f0;
            margin: 0;
            padding: 0;
        }
        #main {
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #fff;
            border-radius: 10px;
            box-shadow: 0px 4px 10px rgba(0, 0, 0, 0.1);
        }
        h1 {
            color: #ff0000;
        }
        .button {
            padding: 20px 40px;
            font-size: 36px;
            margin: 50px;
            cursor: pointer;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 15px;
        }
    </style>
</head>
<body>
    <div id="main">
        <h1>Emergency State</h1>
        <form id="F1" action="reset" method="POST">
            <input type="submit" value="reset" class="button">
        </form>
    </div>
</body>
</html>
)";
