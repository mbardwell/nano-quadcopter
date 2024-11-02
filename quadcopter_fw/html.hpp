#pragma once

#include <Arduino.h>

const String html_header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";

struct Telemetry {
    float I;
    float V;
    float alt;
    bool alt_cal;
    bool imu_cal;
};

auto html_running = [](const Telemetry &data) -> String {
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
            padding: 20px 40px;
            font-size: 24px;
            margin: 20px;
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
            margin: 20px 0;
        }
        input[type="number"] {
            padding: 10px;
            font-size: 18px;
            margin: 10px;
            width: 100%;
            max-width: 300px;
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
            <br>
            <input type="submit" value="set_motor_value" class="button blue-button">
        </form>
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
