
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "web_tune.h"
#include "pid.h"

static const char* WIFI_NAME = "...";
static const char* WIFI_PASS = "SET_A_UNIQUE_PASSWORD";

static WebServer server(80);

// Giao dien web
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

<script>
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