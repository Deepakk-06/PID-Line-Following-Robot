/*
 * ================================================================
 *          PID LINE FOLLOWING ROBOT  —  ESP32 BUILD
 *          Author  : Deepak K  (@Deepakk-06)
 *          Board   : ESP32 DevKit V1
 *          Driver  : TB6612FNG Dual H-Bridge
 *          Sensors : 8-channel IR analog array
 *          Control : Discrete PID | Dual-core | Live WiFi tuning
 * ================================================================
 *
 * SYSTEM OVERVIEW
 * ---------------
 * An 8-channel IR sensor array continuously samples the line position.
 * Each sensor is assigned a positional weight. The weighted average of
 * all active sensor readings produces a signed error value:
 *
 *      Sensor  :  S1   S2   S3   S4   S5   S6   S7   S8
 *      Weight  :  +4   +3   +2   +1   -1   -2   -3   -4
 *
 *      error > 0  →  line is left of center  →  steer left
 *      error < 0  →  line is right of center →  steer right
 *      error = 0  →  robot is centered       →  full speed ahead
 *
 * DUAL CORE ARCHITECTURE
 * ----------------------
 * Core 1 (main loop) — PID control loop runs here uninterrupted.
 *                       Sensor reads, PID compute, motor output.
 *                       No WiFi or serial blocking this core.
 *
 * Core 0 (Task)      — WiFi web server runs here independently.
 *                       Hosts a browser dashboard for live PID
 *                       gain tuning without reflashing firmware.
 *
 * LIVE TUNING VIA WiFi
 * --------------------
 * On boot the ESP32 creates a WiFi access point:
 *   SSID     : PID-LineFollower
 *   Password : deepak123
 * Connect from your phone/laptop and open  http://192.168.4.1
 * Adjust Kp, Kd, Ki and speed sliders live while the robot runs.
 *
 * PWM ON ESP32
 * ------------
 * ESP32 has no analogWrite(). PWM is configured via the LEDC
 * peripheral — 8-bit resolution, 1kHz frequency, 4 channels.
 * All motor speed writes go through ledcWrite().
 *
 * ADC ON ESP32
 * ------------
 * ESP32 ADC is 12-bit (0–4095). Sensor values are mapped to
 * 0–1000 for consistency with the PID weight scale.
 *
 * ================================================================
 */

#include <WiFi.h>
#include <WebServer.h>

// ----------------------------------------------------------------
//  WiFi Access Point Credentials
// ----------------------------------------------------------------
const char* AP_SSID = "PID-LineFollower";
const char* AP_PASS = "deepak123";
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  Motor Driver Pin Map  (TB6612FNG)
// ----------------------------------------------------------------
#define AIN1  25   // Motor A direction pin 1
#define AIN2  26   // Motor A direction pin 2
#define BIN1  27   // Motor B direction pin 1
#define BIN2  14   // Motor B direction pin 2
#define STBY  12   // Driver standby (active HIGH)

// LEDC PWM channel assignments
#define PWMA_CH  0   // Motor A PWM → GPIO 32
#define PWMB_CH  1   // Motor B PWM → GPIO 33
#define PWMA_PIN 32
#define PWMB_PIN 33

// PWM config
#define PWM_FREQ  1000   // 1 kHz
#define PWM_RES   8      // 8-bit → 0–255 range (matches Nano code)
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  IR Sensor GPIO Map  (use ADC1 pins only — ADC2 conflicts WiFi)
// ----------------------------------------------------------------
const int sensorPin[8] = { 36, 39, 34, 35, 32, 33, 4, 2 };
//  Note: GPIO 36, 39, 34, 35 are input-only on ESP32 — fine for ADC
//  Avoid ADC2 pins (0,2,4,12,13,14,15,25,26,27) when WiFi is active
//  If you need all 8 clean ADC1 channels: 32,33,34,35,36,39,4,2
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  User Interface Pins
// ----------------------------------------------------------------
#define BTN_CALIBRATE  18   // Button 1 → start calibration
#define BTN_RUN        19   // Button 2 → start run
#define LED_ONLINE     2    // Onboard LED → on-line indicator
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  Track & Sensor Configuration
// ----------------------------------------------------------------
bool         isBlackLine   = true;   // true = black line on white
unsigned int numSensors    = 8;      // active sensor count
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  PID Gains  — tuned for 15mm black line, smooth curves, speed 220
//  These are declared volatile so the WiFi core can update them
//  safely while the control core reads them.
// ----------------------------------------------------------------
volatile float Kp = 0.13;
volatile float Kd = 0.40;
volatile float Ki = 0.0005;
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  Speed Configuration
// ----------------------------------------------------------------
volatile int lfSpeed      = 220;   // cruise speed (0–255)
volatile int currentSpeed = 60;    // start speed — ramps to lfSpeed
// ----------------------------------------------------------------

// PID state
int    P, D, PIDvalue;
int    I             = 0;
int    previousError = 0;
double error         = 0;
int    lsp, rsp, activeSensors;
int    onLine = 1;

// Positional weights — left = positive error, right = negative
int sensorWeight[8] = { 4, 3, 2, 1, -1, -2, -3, -4 };

// Per-sensor calibration and live readings
int minValues[8], maxValues[8], threshold[8];
int sensorValue[8], sensorArray[8];

// Web server on port 80 (runs on Core 0)
WebServer server(80);


// ================================================================
//  WiFi TUNING DASHBOARD  (served at http://192.168.4.1)
// ================================================================
void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html><html><head>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>PID Tuner</title>
<style>
  body{font-family:monospace;background:#0d0d0d;color:#e0e0e0;padding:20px;max-width:420px;margin:auto}
  h2{color:#00e5ff;margin-bottom:4px}
  p.sub{color:#666;font-size:12px;margin-top:0}
  label{display:block;margin-top:16px;font-size:13px;color:#aaa}
  input[type=range]{width:100%;accent-color:#00e5ff}
  .val{color:#00e5ff;font-weight:bold;font-size:15px}
  button{margin-top:24px;width:100%;padding:12px;background:#00e5ff;
         color:#000;border:none;font-size:15px;font-weight:bold;
         border-radius:6px;cursor:pointer}
  button:active{opacity:0.8}
  .row{display:flex;justify-content:space-between;align-items:center}
</style></head><body>
<h2>PID Line Follower</h2>
<p class='sub'>Live tuning — changes apply instantly</p>
<form action='/set' method='GET'>
  <label>Kp — Proportional
    <div class='row'>
      <input type='range' name='kp' min='0.01' max='0.50' step='0.01'
             value='__KP__' oninput='this.nextElementSibling.textContent=this.value'>
      <span class='val'>__KP__</span>
    </div>
  </label>
  <label>Kd — Derivative (damping)
    <div class='row'>
      <input type='range' name='kd' min='0.00' max='1.00' step='0.01'
             value='__KD__' oninput='this.nextElementSibling.textContent=this.value'>
      <span class='val'>__KD__</span>
    </div>
  </label>
  <label>Ki — Integral (drift fix)
    <div class='row'>
      <input type='range' name='ki' min='0.0000' max='0.005' step='0.0001'
             value='__KI__' oninput='this.nextElementSibling.textContent=this.value'>
      <span class='val'>__KI__</span>
    </div>
  </label>
  <label>Top Speed (0–255)
    <div class='row'>
      <input type='range' name='spd' min='50' max='255' step='1'
             value='__SPD__' oninput='this.nextElementSibling.textContent=this.value'>
      <span class='val'>__SPD__</span>
    </div>
  </label>
  <button type='submit'>Apply</button>
</form>
</body></html>
)rawhtml";

  html.replace("__KP__",  String(Kp,   4));
  html.replace("__KD__",  String(Kd,   4));
  html.replace("__KI__",  String(Ki,   4));
  html.replace("__SPD__", String(lfSpeed));
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("kp"))  Kp       = server.arg("kp").toFloat();
  if (server.hasArg("kd"))  Kd       = server.arg("kd").toFloat();
  if (server.hasArg("ki"))  Ki       = server.arg("ki").toFloat();
  if (server.hasArg("spd")) lfSpeed  = server.arg("spd").toInt();
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// WiFi + web server task — pinned to Core 0
void wifiTask(void* param) {
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[WiFi] AP started — connect to: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",    handleRoot);
  server.on("/set", handleSet);
  server.begin();

  while (1) {
    server.handleClient();
    vTaskDelay(1);   // yield to RTOS — keeps WiFi stack healthy
  }
}


// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] PID Line Following Robot — ESP32");

  // Motor driver GPIO
  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Configure LEDC PWM channels for motor speed control
  ledcSetup(PWMA_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA_PIN, PWMA_CH);
  ledcSetup(PWMB_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB_PIN, PWMB_CH);

  // User interface
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_RUN,       INPUT_PULLUP);
  pinMode(LED_ONLINE,    OUTPUT);

  // Start WiFi tuning server on Core 0, stack 4KB, priority 1
  xTaskCreatePinnedToCore(wifiTask, "wifiTask", 4096, NULL, 1, NULL, 0);

  Serial.println("[BOOT] Ready. Press Button 1 to calibrate.");
}


// ================================================================
//  MAIN LOOP  (runs on Core 1 — unaffected by WiFi)
// ================================================================
void loop() {

  // Wait for calibration trigger
  while (digitalRead(BTN_CALIBRATE)) {}
  delay(1000);
  calibrate();

  // Wait for run trigger
  while (digitalRead(BTN_RUN)) {}
  delay(1000);

  // Reset PID state before run
  I             = 0;
  previousError = 0;
  currentSpeed  = 60;

  Serial.println("[RUN] Starting...");

  while (1) {
    readLine();

    if (currentSpeed < lfSpeed) currentSpeed++;

    if (onLine) {
      lineFollow();
      digitalWrite(LED_ONLINE, HIGH);
    } else {
      digitalWrite(LED_ONLINE, LOW);
      I = 0;   // reset integrator — prevent windup during recovery
      if (error > 0) {
        motor1run(-80);
        motor2run(lfSpeed);
      } else {
        motor1run(lfSpeed);
        motor2run(-80);
      }
    }
  }
}


// ================================================================
//  PID LINE FOLLOW
//  Weighted position error → PID output → differential motor speed
// ================================================================
void lineFollow() {
  error         = 0;
  activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    error         += (double)sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) error /= activeSensors;

  P = error;
  I = constrain(I + (int)error, -500, 500);
  D = (int)error - previousError;

  PIDvalue      = (int)((Kp * P) + (Ki * I) + (Kd * D));
  previousError = (int)error;

  lsp = constrain(currentSpeed - PIDvalue, 0, 255);
  rsp = constrain(currentSpeed + PIDvalue, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}


// ================================================================
//  CALIBRATION
//  Bidirectional sweep — per-sensor min/max → threshold = midpoint
// ================================================================
void calibrate() {
  Serial.println("[CAL] Starting calibration sweep...");

  for (int i = 0; i < numSensors; i++) {
    minValues[i] = analogRead(sensorPin[i]);
    maxValues[i] = analogRead(sensorPin[i]);
  }

  for (int j = 0; j < 5000; j++) { motor1run(60);  motor2run(-60); updateCalibration(); }
  for (int j = 0; j < 5000; j++) { motor1run(-60); motor2run(60);  updateCalibration(); }

  motor1run(0);
  motor2run(0);

  Serial.print("[CAL] Thresholds: ");
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println("\n[CAL] Done. Press Button 2 to run.");
}

void updateCalibration() {
  for (int i = 0; i < numSensors; i++) {
    int v = analogRead(sensorPin[i]);
    if (v < minValues[i]) minValues[i] = v;
    if (v > maxValues[i]) maxValues[i] = v;
  }
}


// ================================================================
//  READ LINE
//  ESP32 ADC is 12-bit (0–4095) → mapped to 0–1000
// ================================================================
void readLine() {
  onLine = 0;
  for (int i = 0; i < numSensors; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(analogRead(sensorPin[i]), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(analogRead(sensorPin[i]), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500) ? 1 : 0;
    if (sensorArray[i]) onLine = 1;
  }
}


// ================================================================
//  MOTOR CONTROL  (ESP32 uses ledcWrite instead of analogWrite)
//  Positive = forward, negative = reverse, zero = coast
// ================================================================
void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(PWMA_CH, spd);
  } else if (spd < 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA_CH, abs(spd));
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
    ledcWrite(PWMA_CH, 0);
  }
}

void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    ledcWrite(PWMB_CH, spd);
  } else if (spd < 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB_CH, abs(spd));
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
    ledcWrite(PWMB_CH, 0);
  }
}
