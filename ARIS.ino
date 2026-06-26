/*
   PROJECT: A.R.I.S. (Autonomous Rescue & Inspection System)
   THREAD: Robotics and Autonomous Systems (Innovate ECE)
   HARDWARE: ESP32, L298N (4WD), GY-521 (MPU6050), HC-SR04, MQ-6 Gas Sensor
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- WIFI CONFIG ---
const char* ssid = "ARIS_Telemetry";
const char* password = "innovate_ece";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- PIN DEFINITIONS ---
#define ENA 13
#define IN1 33
#define IN2 14
#define IN3 27
#define IN4 26
#define ENB 25
#define TRIG_PIN 32
#define ECHO_PIN 18
#define GAS_PIN 36

// --- GLOBAL OBJECTS & VARIABLES ---
Adafruit_MPU6050 mpu;
enum State { EXPLORING = 0, TURNING = 1, HAZARD_DETECTED = 2 };
State robotState = EXPLORING;

// Sensor Data
int currentDist = 0;
int gasLevel = 0;
int gasThreshold = 1800;  // TUNE THIS: 0-4095 scale on ESP32

// Navigation Data
float currentYaw = 0.0;
float targetHeading = 0.0;
float gyroZ_offset = 0.0;
unsigned long lastTime = 0;
float Kp = 2.5;  // Proportional Gain
int baseSpeed = 255;

// Telemetry Timer
unsigned long lastWsUpdate = 0;

// --- HTML DASHBOARD (WITH JAVASCRIPT WEBSOCKETS) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>A.R.I.S. Command Center</title>
  <style>
    :root { --bg: #121212; --card: #1e1e24; --accent: #eab308; --safe: #22c55e; --alert: #ef4444; --text: #f3f4f6; }
    body { font-family: 'Segoe UI', Roboto, Helvetica, Arial, sans-serif; background-color: var(--bg); color: var(--text); text-align: center; margin: 0; padding: 20px; }
    h1 { margin-bottom: 5px; color: var(--accent); letter-spacing: 2px; text-transform: uppercase; font-size: 1.8rem;}
    p.subtitle { margin-top: 0; color: #9ca3af; font-size: 0.9em; margin-bottom: 25px; text-transform: uppercase; letter-spacing: 1px;}
    
    /* Layout */
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; max-width: 900px; margin: 0 auto; }
    .card { background: var(--card); padding: 25px; border-radius: 12px; box-shadow: 0 8px 16px rgba(0,0,0,0.4); border-top: 4px solid #333; transition: transform 0.2s; }
    .card:hover { transform: translateY(-2px); }
    .card h3 { margin-top: 0; color: #9ca3af; font-size: 0.9rem; text-transform: uppercase; letter-spacing: 1.5px; }
    .value { font-size: 2.5rem; font-weight: bold; margin: 15px 0; font-family: monospace;}
    
    /* Status Badge Animations */
    .status-badge { display: inline-block; padding: 12px 25px; border-radius: 30px; font-weight: bold; font-size: 1.2rem; letter-spacing: 2px; margin-top: 10px;}
    .safe-bg { background: rgba(34, 197, 94, 0.15); color: var(--safe); border: 1px solid var(--safe); }
    .alert-bg { background: rgba(239, 68, 68, 0.15); color: var(--alert); border: 1px solid var(--alert); animation: pulse 1.5s infinite; }
    .scan-bg { background: rgba(234, 179, 8, 0.15); color: var(--accent); border: 1px solid var(--accent); }
    
    @keyframes pulse { 
      0% { box-shadow: 0 0 0 0 rgba(239,68,68, 0.5); } 
      70% { box-shadow: 0 0 0 15px rgba(239,68,68, 0); } 
      100% { box-shadow: 0 0 0 0 rgba(239,68,68, 0); } 
    }

    /* Live Compass */
    .compass-container { position: relative; width: 100px; height: 100px; margin: 0 auto; border: 3px solid #4b5563; border-radius: 50%; display: flex; align-items: center; justify-content: center; background: #111; box-shadow: inset 0 0 15px rgba(0,0,0,0.8);}
    .compass-arrow { width: 0; height: 0; border-left: 10px solid transparent; border-right: 10px solid transparent; border-bottom: 45px solid var(--accent); position: absolute; top: 5px; transition: transform 0.1s ease-out; transform-origin: 50% 45px; }
    
    /* Telemetry Bars */
    .bar-bg { width: 100%; background: #374151; border-radius: 8px; height: 12px; overflow: hidden; margin-top: 15px; box-shadow: inset 0 2px 4px rgba(0,0,0,0.5);}
    .bar-fill { height: 100%; width: 0%; transition: width 0.3s ease, background-color 0.3s ease; }
  </style>
</head>
<body>
  <h1>A.R.I.S. Command Center</h1>
  <p class="subtitle">Live Telemetry Dashboard</p>
  
  <div class="grid">
    <div class="card" style="border-top-color: var(--accent); grid-column: 1 / -1;">
      <h3>System State</h3>
      <div id="status" class="status-badge safe-bg">CONNECTING...</div>
    </div>

    <div class="card" style="border-top-color: #3b82f6;">
      <h3>IMU Heading</h3>
      <div class="compass-container">
        <div id="compass-arrow" class="compass-arrow"></div>
      </div>
      <div class="value"><span id="yaw">0</span>&deg;</div>
    </div>

    <div class="card" style="border-top-color: var(--safe);">
      <h3>Sonar Clearance</h3>
      <div class="value"><span id="dist">0</span> <span style="font-size: 1rem; color: #9ca3af;">cm</span></div>
      <div class="bar-bg"><div id="dist-bar" class="bar-fill" style="background: var(--safe);"></div></div>
    </div>

    <div class="card" style="border-top-color: var(--alert);">
      <h3>Hazard Air Quality</h3>
      <div class="value"><span id="gas">0</span> <span style="font-size: 1rem; color: #9ca3af;">ADC</span></div>
      <div class="bar-bg"><div id="gas-bar" class="bar-fill" style="background: var(--safe);"></div></div>
    </div>
  </div>

  <script>
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    window.addEventListener('load', initWebSocket);
    
    function initWebSocket() {
      websocket = new WebSocket(gateway);
      websocket.onclose = function() { setTimeout(initWebSocket, 2000); };
      websocket.onmessage = onMessage;
    }
    
    function onMessage(event) {
      var data = JSON.parse(event.data);
      
      // 1. Update Raw Text Values
      document.getElementById('gas').innerText = data.gas;
      document.getElementById('dist').innerText = data.dist;
      document.getElementById('yaw').innerText = data.yaw;
      
      // 2. Update Live Compass Rotation
      document.getElementById('compass-arrow').style.transform = `rotate(${data.yaw}deg)`;
      
      // 3. Update Distance Bar (Fills up as clearance increases, turns red if under 25cm)
      var distPercent = Math.min((data.dist / 150) * 100, 100); // 150cm max visual scale
      var distBar = document.getElementById('dist-bar');
      distBar.style.width = distPercent + '%';
      distBar.style.background = data.dist < 25 ? 'var(--alert)' : 'var(--safe)';
      
      // 4. Update Gas Bar (Fills up as gas increases, turns red if over threshold)
      var gasPercent = Math.min((data.gas / 4095) * 100, 100);
      var gasBar = document.getElementById('gas-bar');
      gasBar.style.width = gasPercent + '%';
      gasBar.style.background = data.gas > 1800 ? 'var(--alert)' : 'var(--accent)';
      
      // 5. Update Status Badge Color and Text
      var statusEl = document.getElementById('status');
      if (data.state == 0) {
        statusEl.innerText = "EXPLORING";
        statusEl.className = "status-badge safe-bg";
      } else if (data.state == 1) {
        statusEl.innerText = "EVASIVE MANEUVER";
        statusEl.className = "status-badge scan-bg";
      } else {
        statusEl.innerText = "HAZARD DETECTED!";
        statusEl.className = "status-badge alert-bg";
      }
    }
  </script>
</body>
</html>
)rawliteral";

// --- FUNCTIONS ---

void notifyClients() {
  // Construct JSON payload for the WebSocket
  String json = "{\"state\":" + String(robotState) + 
                ",\"gas\":" + String(gasLevel) + 
                ",\"dist\":" + String(currentDist) + 
                ",\"yaw\":" + String((int)currentYaw) + "}";
  ws.textAll(json);
}

void calibrateIMU() {
  float totalZ = 0.0; 
  sensors_event_t a, g, temp;
  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    totalZ += g.gyro.z;
    delay(10);
  }
  gyroZ_offset = totalZ / 200.0;
}

void updateHeading() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float z_rate = (g.gyro.z - gyroZ_offset) * 57.2958;
  
  // PATCH 1: Inverted gyro math (-=) to fix the Positive Feedback Loop
  if (abs(z_rate) > 1.0) currentYaw -= (z_rate * dt); 
}

int readSonar() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 5000); 
  return (duration == 0) ? 999 : duration * 0.034 / 2;
}

void setMotors(int left, int right) {
  // --- LEFT MOTOR (OUT3 & OUT4) ---
  if (left == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN4, left > 0 ? LOW : HIGH);
    digitalWrite(IN3, left > 0 ? HIGH : LOW);
  }
  analogWrite(ENB, abs(left));

  // --- RIGHT MOTOR (OUT1 & OUT2) ---
  if (right == 0) {
    // PATCH 2: Standardized right motor coasting (LOW, LOW)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, right > 0 ? HIGH : LOW);
    digitalWrite(IN2, right > 0 ? LOW : HIGH);
  }
  analogWrite(ENA, abs(right));
}

void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // WiFi Setup
  WiFi.softAP(ssid, password);
  
  // Async Web Server Setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.addHandler(&ws);
  server.begin();

  // IMU Setup
  Wire.begin(); 
  if (mpu.begin()) {
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("Calibrating IMU in 3 seconds... HANDS OFF THE ROVER!");
    
    // PATCH 3: Safe startup delay to prevent wild gyro offsets
    delay(3000); 
    calibrateIMU();
    Serial.println("Calibration complete. Exploring!");
  }
  lastTime = millis();
}

void loop() {
  // Clean up disconnected WebSocket clients
  ws.cleanupClients();

  // Update Sensors
  updateHeading();
  currentDist = readSonar();
  gasLevel = analogRead(GAS_PIN);

  // --- LOGIC ---
  if (gasLevel > gasThreshold) {
    robotState = HAZARD_DETECTED;
  } else if (robotState != TURNING && currentDist < 25) {
    robotState = TURNING;
    targetHeading = currentYaw - 90.0;  // Set target 90 degrees to the right
  }

  // --- ACTUATION ---
  switch (robotState) {
    case EXPLORING:
      {
        float error = targetHeading - currentYaw;
        float correction = Kp * error;
        setMotors(constrain(baseSpeed - correction, 0, 255), constrain(baseSpeed + correction, 0, 255));
        break;
      }
    case TURNING:
      {
        setMotors(120, -120);
        
        // PATCH 4: Threshold crossing logic & dynamic path clearing
        if (currentYaw <= targetHeading || currentDist > 35) {
          setMotors(0, 0);         
          robotState = EXPLORING;  
        }
        break;
      }
    case HAZARD_DETECTED:
      {
        setMotors(0, 0);
        delay(100);
        setMotors(100, 100);
        delay(100);  
        break;
      }
  }

  // --- TELEMETRY BROADCAST ---
  if (millis() - lastWsUpdate > 250) {
    notifyClients();
    lastWsUpdate = millis();
  }
  
  delay(10);  // Stability pause
}