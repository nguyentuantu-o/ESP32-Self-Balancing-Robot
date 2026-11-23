
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp32-hal-bt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "web_tune.h"
#include "pid.h"

static const char* WIFI_NAME = "...";
static const char* WIFI_PASS = "SET_A_UNIQUE_PASSWORD";

static WebServer server(80);
static IMUData latestIMU = {};
static float latestTargetRoll = 0.0f;
static int latestPWM = 0;
static SpeedData latestSpeed = {};
static bool disarmRequested = false;

// Web interface.
static const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport"
        content="width=device-width, initial-scale=1.0">
  <title>Robot PID Tuner</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111827;
      color: white;
      padding: 20px;
      max-width: 420px;
      margin: auto;
    }
    h2 { text-align: center; }
    .card {
      background: #1f2937;
      padding: 20px;
      border-radius: 15px;
    }
    label {
      display: block;
      margin-top: 16px;
      margin-bottom: 6px;
    }
    input {
      width: 100%;
      box-sizing: border-box;
      padding: 12px;
      border-radius: 8px;
      border: none;
      font-size: 18px;
    }
    button {
      margin-top: 24px;
      width: 100%;
      padding: 14px;
      background: #2563eb;
      color: white;
      border: none;
      border-radius: 8px;
      font-size: 17px;
    }
    #status {
      text-align: center;
      margin-top: 16px;
    }
    .note {
      color: #9ca3af;
      font-size: 13px;
      line-height: 1.5;
    }
    .readings {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 18px;
    }
    .reading {
      background: #111827;
      padding: 10px;
      border-radius: 8px;
      font-size: 14px;
    }
    .reading span {
      color: #60a5fa;
      font-variant-numeric: tabular-nums;
    }
    .secondary {
      background: #4b5563;
    }
    .chart {
      display: block;
      width: 100%;
      height: 220px;
      margin-top: 12px;
      background: #111827;
      border-radius: 8px;
    }
  </style>
</head>
<body>
  <h2>ESP32 PID Tuner</h2>

  <div class="card">
    <label>Kp</label>
    <input type="number" id="kp"
           min="0" max="100" step="0.1">

    <label>Ki</label>
    <input type="number" id="ki"
           min="0" max="100" step="0.01">

    <label>Kd</label>
    <input type="number" id="kd"
           min="0" max="100" step="0.05">

    <button onclick="applyPID()">Apply PID</button>

    <p id="status">Dang tai...</p>
    <p class="note">
      Thay doi co hieu luc ngay.
      Nen tat motor truoc khi chinh PID.
      Cac gia tri khong duoc luu sau reset.
    </p>
  </div>

  <div class="card" style="margin-top: 16px">
    <h3>MPU6050 live data</h3>
    <p class="note">Do thi truc tiep - 10 giay gan nhat, cap nhat moi 100 ms.</p>
    <p class="note">
      Speed loop dang tat an toan. Ke xe len, tat motor va xoay banh theo chieu
      xe tien de kiem tra dau Left/Right raw RPM truoc khi bat.
    </p>
    <canvas id="rollChart" class="chart" aria-label="Roll and target angle chart"></canvas>
    <canvas id="speedChart" class="chart" aria-label="Encoder and speed loop RPM chart"></canvas>
    <canvas id="pwmChart" class="chart" aria-label="PWM chart"></canvas>
    <button class="secondary" onclick="stopMotors()">Disarm motors</button>
    <div class="readings">
      <div class="reading">Roll: <span id="roll">--</span> deg</div>
      <div class="reading">Accel roll: <span id="rollAccel">--</span> deg</div>
      <div class="reading">Gyro roll: <span id="rollGyro">--</span> deg</div>
      <div class="reading">Gyro X: <span id="gyroX">--</span> deg/s</div>
      <div class="reading">Target: <span id="targetRoll">--</span> deg</div>
      <div class="reading">PWM: <span id="pwm">--</span></div>
      <div class="reading">Left raw RPM: <span id="leftRawRPM">--</span></div>
      <div class="reading">Right raw RPM: <span id="rightRawRPM">--</span></div>
      <div class="reading">Speed RPM: <span id="speedRPM">--</span></div>
      <div class="reading">Target RPM: <span id="targetRPM">--</span></div>
      <div class="reading">Accel X: <span id="accelX">--</span> m/s2</div>
      <div class="reading">Accel Y: <span id="accelY">--</span> m/s2</div>
      <div class="reading">Accel Z: <span id="accelZ">--</span> m/s2</div>
      <div class="reading">Gyro Y: <span id="gyroY">--</span> deg/s</div>
      <div class="reading">Gyro Z: <span id="gyroZ">--</span> deg/s</div>
    </div>
    <button id="recordButton" onclick="toggleRecording()">Start recording</button>
    <button class="secondary" onclick="downloadCSV()">Download CSV</button>
    <p id="imuStatus" class="note">Dang ket noi MPU6050...</p>
    <p class="note">
      Do thi cap nhat khong can bat ghi CSV. Ghi CSV lay mau moi 100 ms tren
      trinh duyet; khi thu, giu robot o khu vuc an toan.
    </p>
  </div>

<script>
const dataFields = [
  'roll', 'rollAccel', 'rollGyro', 'gyroRateX',
  'targetRoll', 'pwm', 'leftRawRPM', 'rightRawRPM',
  'speedRPM', 'targetRPM',
  'accelX', 'accelY', 'accelZ', 'gyroRateY', 'gyroRateZ'
];
const dataLabels = [
  'roll_deg', 'roll_accel_deg', 'roll_gyro_deg', 'gyro_x_deg_s',
  'target_deg', 'pwm', 'left_raw_rpm', 'right_raw_rpm',
  'speed_rpm', 'target_rpm',
  'accel_x_m_s2', 'accel_y_m_s2', 'accel_z_m_s2',
  'gyro_y_deg_s', 'gyro_z_deg_s'
];
let recording = false;
let recordingStart = 0;
let samples = [];
let imuRequestPending = false;
const liveStart = performance.now();
const liveSamples = [];
const liveWindowMs = 10000;

function drawLiveChart(canvasId, lines, minY, maxY, unit) {
  const canvas = document.getElementById(canvasId);
  const rect = canvas.getBoundingClientRect();
  if (!rect.width || !rect.height) return;

  const pixelRatio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * pixelRatio);
  canvas.height = Math.round(rect.height * pixelRatio);

  const ctx = canvas.getContext('2d');
  ctx.scale(pixelRatio, pixelRatio);

  const width = rect.width;
  const height = rect.height;
  const left = 42;
  const right = width - 10;
  const top = 12;
  const bottom = height - 24;
  const now = performance.now();
  const start = Math.max(0, now - liveWindowMs - liveStart);
  const end = now - liveStart;
  const colors = ['#60a5fa', '#f97316', '#34d399', '#facc15'];

  ctx.clearRect(0, 0, width, height);
  ctx.font = '11px Arial';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';

  for (let tick = 0; tick <= 4; tick++) {
    const value = maxY - (maxY - minY) * tick / 4;
    const y = top + (bottom - top) * tick / 4;
    ctx.strokeStyle = '#374151';
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();
    ctx.fillStyle = '#d1d5db';
    ctx.fillText(String(Math.round(value)), left - 5, y);
  }

  ctx.strokeStyle = '#9ca3af';
  ctx.beginPath();
  ctx.moveTo(left, top);
  ctx.lineTo(left, bottom);
  ctx.lineTo(right, bottom);
  ctx.stroke();

  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (let tick = 0; tick <= 4; tick++) {
    const x = left + (right - left) * tick / 4;
    const secondsAgo = (end - start) * (4 - tick) / 4000;
    ctx.fillStyle = '#d1d5db';
    ctx.fillText(secondsAgo === 0 ? 'now' : '-' + secondsAgo.toFixed(1) + 's', x, bottom + 5);
  }

  lines.forEach((line, index) => {
    ctx.strokeStyle = colors[index];
    ctx.lineWidth = 2;
    ctx.beginPath();
    let started = false;
    liveSamples.forEach(sample => {
      const time = sample.time - liveStart;
      if (time < start || time > end) return;
      const x = left + (time - start) / Math.max(1, end - start) * (right - left);
      const value = Math.max(minY, Math.min(maxY, sample[line.field]));
      const y = top + (maxY - value) / (maxY - minY) * (bottom - top);
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  });

  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.font = '12px Arial';
  lines.forEach((line, index) => {
    const x = left + index * 100;
    ctx.fillStyle = colors[index];
    ctx.fillRect(x, 2, 12, 3);
    ctx.fillText(line.label, x + 16, 0);
  });
  ctx.fillStyle = '#d1d5db';
  ctx.textAlign = 'right';
  ctx.fillText(unit, right, 0);
}

function drawLiveCharts() {
  drawLiveChart(
    'rollChart',
    [
      {field: 'roll', label: 'Roll'},
      {field: 'targetRoll', label: 'Target'}
    ],
    -30,
    30,
    'deg'
  );
  drawLiveChart(
    'pwmChart',
    [{field: 'pwm', label: 'PWM'}],
    -255,
    255,
    'signed PWM'
  );
  drawLiveChart(
    'speedChart',
    [
      {field: 'leftRawRPM', label: 'Left raw RPM'},
      {field: 'rightRawRPM', label: 'Right raw RPM'},
      {field: 'speedRPM', label: 'Speed'},
      {field: 'targetRPM', label: 'Target RPM'}
    ],
    -300,
    300,
    'RPM'
  );
}

async function loadPID() {
  try {
    const r = await fetch('/pid');
    if (!r.ok) throw Error('Connection error');

    const d = await r.json();
    document.getElementById('kp').value = d.kp;
    document.getElementById('ki').value = d.ki;
    document.getElementById('kd').value = d.kd;
    document.getElementById('status').textContent =
      'Ready';
  } catch (e) {
    document.getElementById('status').textContent =
      'Connection error';
  }
}

async function updateIMU() {
  if (imuRequestPending) return;
  imuRequestPending = true;
  try {
    const r = await fetch('/imu', {cache: 'no-store'});
    if (!r.ok) throw Error('Connection error');

    const d = await r.json();
    const sample = {
      time: performance.now(),
      roll: Number(d.roll),
      targetRoll: Number(d.targetRoll),
      leftRawRPM: Number(d.leftRawRPM),
      rightRawRPM: Number(d.rightRawRPM),
      speedRPM: Number(d.speedRPM),
      targetRPM: Number(d.targetRPM),
      pwm: Number(d.pwm)
    };
    if (Object.values(sample).some(value => !Number.isFinite(value))) {
      throw Error('Invalid sensor data');
    }
    liveSamples.push(sample);
    while (liveSamples.length && liveSamples[0].time < sample.time - liveWindowMs) {
      liveSamples.shift();
    }
    drawLiveCharts();

    const elementIds = {
      roll: 'roll',
      rollAccel: 'rollAccel',
      rollGyro: 'rollGyro',
      gyroRateX: 'gyroX',
      targetRoll: 'targetRoll',
      pwm: 'pwm',
      leftRawRPM: 'leftRawRPM',
      rightRawRPM: 'rightRawRPM',
      speedRPM: 'speedRPM',
      targetRPM: 'targetRPM',
      accelX: 'accelX',
      accelY: 'accelY',
      accelZ: 'accelZ',
      gyroRateY: 'gyroY',
      gyroRateZ: 'gyroZ'
    };
    for (const field of dataFields) {
      document.getElementById(elementIds[field]).textContent =
        Number(d[field]).toFixed(2);
    }

    if (recording) {
      samples.push([
        performance.now() - recordingStart,
        ...dataFields.map(field => d[field])
      ]);
      if (samples.length >= 10000) {
        recording = false;
        document.getElementById('recordButton').textContent =
          'Start recording';
        document.getElementById('imuStatus').textContent =
          'Da dat gioi han 10000 mau; tai CSV de luu du lieu.';
      } else {
        document.getElementById('imuStatus').textContent =
          'Dang ghi ' + samples.length + ' mau';
      }
    } else {
      document.getElementById('imuStatus').textContent =
        'Live - ' + samples.length + ' mau da ghi';
    }
  } catch (e) {
    document.getElementById('imuStatus').textContent =
      'Loi doc MPU6050: ' + e.message;
  } finally {
    imuRequestPending = false;
  }
}

function toggleRecording() {
  recording = !recording;
  if (recording) {
    samples = [];
    recordingStart = performance.now();
  }
  document.getElementById('recordButton').textContent =
    recording ? 'Stop recording' : 'Start recording';
  document.getElementById('imuStatus').textContent =
    recording ? 'Dang ghi du lieu...' : 'Da dung ghi - ' + samples.length + ' mau';
}

function downloadCSV() {
  if (samples.length === 0) {
    document.getElementById('imuStatus').textContent =
      'Chua co du lieu. Bam Start recording truoc.';
    return;
  }
  const csv = [
    ['time_ms', ...dataLabels].join(','),
    ...samples.map(sample => sample.map(value => Number(value).toFixed(4)).join(','))
  ].join('\r\n');
  const url = URL.createObjectURL(new Blob([csv], {type: 'text/csv'}));
  const link = document.createElement('a');
  link.href = url;
  link.download = 'mpu6050-data.csv';
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

async function stopMotors() {
  try {
    const r = await fetch('/disarm', {method: 'POST'});
    if (!r.ok) throw Error(await r.text());
    document.getElementById('imuStatus').textContent =
      'Motor da tat; auto-arm se khong bat lai cho den khi reset.';
  } catch (e) {
    document.getElementById('imuStatus').textContent =
      'Khong tat duoc motor: ' + e.message;
  }
}

async function applyPID() {
  const kp = document.getElementById('kp').value;
  const ki = document.getElementById('ki').value;
  const kd = document.getElementById('kd').value;

  const data = new URLSearchParams({kp, ki, kd});

  try {
    const r = await fetch('/set', {
      method: 'POST',
      body: data
    });

    if (!r.ok) throw Error(await r.text());

    document.getElementById('status').textContent =
      'PID Updated!';
  } catch (e) {
    document.getElementById('status').textContent =
      'Error: ' + e.message;
  }
}

loadPID();
updateIMU();
setInterval(updateIMU, 100);
window.addEventListener('resize', drawLiveCharts);
</script>
</body>
</html>
)rawliteral";

static bool readGain(const String &name,
                     float &value,
                     float maximum) {
    if (!server.hasArg(name)) return false;

    String input = server.arg(name);
    const char* start = input.c_str();
    char* end = nullptr;

    value = strtof(start, &end);

    if (end == start || *end != '\0') return false;
    if (!isfinite(value)) return false;
    if (value < 0.0f || value > maximum) return false;

    return true;
}

void webTuneInit() {
    if (strcmp(WIFI_PASS, "SET_A_UNIQUE_PASSWORD") == 0) {
        Serial.println("Set a unique WiFi password before enabling the web tuner.");
        return;
    }

    btStop();
    WiFi.mode(WIFI_AP);

    if (!WiFi.softAP(WIFI_NAME, WIFI_PASS)) {
        Serial.println("WiFi AP ERROR!");
        return;
    }

    Serial.print("PID Web: http://");
    Serial.println(WiFi.softAPIP());

   server.on("/", HTTP_GET, []() {
    Serial.println("PHONE REQUEST /");

    server.sendHeader("Connection", "close");
    server.send_P(
        200,
        "text/html; charset=utf-8",
        PAGE,
        strlen_P(PAGE)
    );

    Serial.println("HTML SENT");
});

    server.on("/pid", HTTP_GET, []() {
        float kp, ki, kd;
        getPIDGains(kp, ki, kd);

        String json = "{\"kp\":" + String(kp, 2)
                    + ",\"ki\":" + String(ki, 3)
                    + ",\"kd\":" + String(kd, 3) + "}";

        server.send(200, "application/json", json);
    });

    server.on("/imu", HTTP_GET, []() {
        const String json = "{\"rollAccel\":" + String(latestIMU.rollAccel, 3)
                          + ",\"rollGyro\":" + String(latestIMU.rollGyro, 3)
                          + ",\"roll\":" + String(latestIMU.roll, 3)
                          + ",\"gyroRateX\":" + String(latestIMU.gyroRateX, 3)
                          + ",\"targetRoll\":" + String(latestTargetRoll, 3)
                          + ",\"pwm\":" + String(latestPWM)
                          + ",\"leftRawRPM\":" + String(latestSpeed.leftRawRPM, 3)
                          + ",\"rightRawRPM\":" + String(latestSpeed.rightRawRPM, 3)
                          + ",\"speedRPM\":" + String(latestSpeed.speedRPM, 3)
                          + ",\"targetRPM\":" + String(latestSpeed.targetRPM, 3)
                          + ",\"accelX\":" + String(latestIMU.accelX, 3)
                          + ",\"accelY\":" + String(latestIMU.accelY, 3)
                          + ",\"accelZ\":" + String(latestIMU.accelZ, 3)
                          + ",\"gyroRateY\":" + String(latestIMU.gyroRateY, 3)
                          + ",\"gyroRateZ\":" + String(latestIMU.gyroRateZ, 3)
                          + "}";

        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", json);
    });

    server.on("/disarm", HTTP_POST, []() {
        disarmRequested = true;
        server.send(200, "text/plain", "Motor disarm requested");
    });

    server.on("/set", HTTP_POST, []() {
        float kp, ki, kd;
if (!readGain("kp", kp, 100.0f) ||
    !readGain("ki", ki, 100.0f) ||
    !readGain("kd", kd, 100.0f)) {

    server.send(400, "text/plain",
                "Invalid PID values");
    return;
}

        setPIDGains(kp, ki, kd);

        Serial.printf(
            "PID UPDATED: Kp=%.2f Ki=%.2f Kd=%.2f\n",
            kp, ki, kd
        );

        server.send(200, "text/plain", "OK");
    });

    server.begin();
    Serial.println("Web PID ready!");
}

void webTuneSetIMUData(
    const IMUData &data,
    float targetRoll,
    int pwm,
    const SpeedData &speed
) {
    latestIMU = data;
    latestTargetRoll = targetRoll;
    latestPWM = pwm;
    latestSpeed = speed;
}

bool webTuneTakeDisarmRequest() {
    const bool requested = disarmRequested;
    disarmRequested = false;
    return requested;
}

void webTuneHandle() {
    static unsigned long lastCheck = 0;

    server.handleClient();

    if (millis() - lastCheck >= 2000) {
        lastCheck = millis();

        Serial.printf(
            "WiFi clients: %d | Free heap: %u\n",
            WiFi.softAPgetStationNum(),
            ESP.getFreeHeap()
        );
    }
}