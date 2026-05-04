/*
 * ================================================================
 *          PID LINE FOLLOWING ROBOT
 *          Author  : Deepak K  (@Deepakk-06)
 *          Control : Discrete PID with anti-windup & auto-calibration
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
 * The PID controller converts that error into a differential speed
 * correction applied to the two drive motors:
 *
 *      leftSpeed  = baseSpeed - PIDoutput
 *      rightSpeed = baseSpeed + PIDoutput
 *
 * CALIBRATION
 * -----------
 * Before each run the robot sweeps left and right, recording the min
 * and max analog value seen by every sensor. The midpoint becomes
 * each sensor's individual binary threshold. This compensates for
 * uneven lighting, sensor variation, and surface reflectivity.
 *
 * LINE LOSS RECOVERY
 * ------------------
 * If all sensors go inactive (robot has left the line), the integrator
 * is reset to prevent windup and the robot spins hard in the direction
 * of the last known error until the line is reacquired.
 *
 * ================================================================
 */

// Fast ADC access macros (set/clear individual bits in SFR registers)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ----------------------------------------------------------------
//  Motor Driver Pin Map  (TB6612FNG)
// ----------------------------------------------------------------
#define AIN1  4    // Motor A direction pin 1
#define AIN2  3    // Motor A direction pin 2
#define PWMA  9    // Motor A PWM speed
#define BIN1  6    // Motor B direction pin 1
#define BIN2  7    // Motor B direction pin 2
#define PWMB  10   // Motor B PWM speed
#define STBY  5    // Driver standby (active HIGH)
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  Track & Sensor Configuration  — edit these to match your setup
// ----------------------------------------------------------------
bool         isBlackLine    = true;  // true = black line on white surface
unsigned int lineThickness  = 15;    // line width in mm (keep between 10–35)
unsigned int numSensors     = 8;     // number of active IR sensors (5, 7, or 8)
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  PID Gains  — tuned for 15mm black line, smooth curves, speed 220
//
//  Kp  Proportional: steering effort per unit of error.
//       Too high → oscillation. Too low → sluggish cornering.
//
//  Kd  Derivative: resists rapid error changes (damps oscillation).
//       The most impactful gain for high-speed stability.
//       Increase this first if the robot wiggles at speed.
//
//  Ki  Integral: accumulates residual error over time.
//       Fixes steady-state drift on long straights.
//       Keep small — large values cause integrator windup.
// ----------------------------------------------------------------
float Kp = 0.13;
float Kd = 0.40;
float Ki = 0.0005;
// ----------------------------------------------------------------

// ----------------------------------------------------------------
//  Speed Configuration
// ----------------------------------------------------------------
int lfSpeed      = 220;   // cruise speed (0–255)
int currentSpeed = 60;    // starting speed — ramps up to lfSpeed each loop
// ----------------------------------------------------------------

// PID state variables
int    P, D, PIDvalue;
int    I             = 0;
int    previousError = 0;
double error         = 0;
int    lsp, rsp, activeSensors;
int    onLine = 1;

// Positional weights for 8-sensor array (left = positive, right = negative)
int sensorWeight[8] = { 4, 3, 2, 1, -1, -2, -3, -4 };

// Per-sensor calibration and live readings
int minValues[8], maxValues[8], threshold[8];
int sensorValue[8], sensorArray[8];


// ================================================================
//  SETUP
// ================================================================
void setup() {

  // Set ADC prescaler to 16 → ADC clock ~1MHz → faster analogRead
  // Default prescaler is 128 (~125kHz). With 8 sensors sampled
  // every loop iteration, faster ADC directly improves loop frequency.
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  // Motor driver GPIO
  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // take driver out of standby

  // User interface
  pinMode(11, INPUT_PULLUP);  // Button 1 → start calibration
  pinMode(12, INPUT_PULLUP);  // Button 2 → start run
  pinMode(13, OUTPUT);        // LED → on-line indicator

  lineThickness = constrain(lineThickness, 10, 35);
}


// ================================================================
//  MAIN LOOP
// ================================================================
void loop() {

  // --- Wait for calibration trigger ---
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();

  // --- Wait for run trigger ---
  while (digitalRead(12)) {}
  delay(1000);

  // Reset PID state before run to avoid stale values
  I            = 0;
  previousError = 0;
  currentSpeed  = 60;

  // --- Main control loop ---
  while (1) {

    readLine();

    // Gradually ramp speed up to cruise — avoids wheel slip on start
    if (currentSpeed < lfSpeed) currentSpeed++;

    if (onLine) {
      // On line: run PID and drive
      lineFollow();
      digitalWrite(13, HIGH);

    } else {
      // Line lost: reset integrator, spin toward last known error
      digitalWrite(13, LOW);
      I = 0;

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
//  Computes weighted position error from active sensors,
//  runs the PID equation, and sets differential motor speeds.
// ================================================================
void lineFollow() {

  error         = 0;
  activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    error         += (double)sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  // Weighted average — normalises error regardless of how many
  // sensors are active at a given moment
  if (activeSensors > 0) error /= activeSensors;

  P = error;
  I = constrain(I + (int)error, -500, 500);  // anti-windup clamp
  D = (int)error - previousError;

  PIDvalue     = (int)((Kp * P) + (Ki * I) + (Kd * D));
  previousError = (int)error;

  lsp = constrain(currentSpeed - PIDvalue, 0, 255);
  rsp = constrain(currentSpeed + PIDvalue, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}


// ================================================================
//  CALIBRATION
//  Sweeps right then left, recording per-sensor min/max values.
//  Threshold = midpoint of each sensor's individual dynamic range.
//  Bidirectional sweep ensures every sensor crosses both the line
//  and the background surface regardless of sensor position.
// ================================================================
void calibrate() {

  Serial.println("[CAL] Starting calibration sweep...");

  // Seed min/max with first reading
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  // Phase 1 — sweep right
  for (int j = 0; j < 5000; j++) {
    motor1run(60);
    motor2run(-60);
    updateCalibration();
  }

  // Phase 2 — sweep left (return to start, extends sensor range)
  for (int j = 0; j < 5000; j++) {
    motor1run(-60);
    motor2run(60);
    updateCalibration();
  }

  motor1run(0);
  motor2run(0);

  // Compute thresholds and report over serial
  Serial.print("[CAL] Thresholds: ");
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println("\n[CAL] Done. Press Button 2 to run.");
}


// Helper — called inside calibration loops to keep them readable
void updateCalibration() {
  for (int i = 0; i < numSensors; i++) {
    int v = analogRead(i);
    if (v < minValues[i]) minValues[i] = v;
    if (v > maxValues[i]) maxValues[i] = v;
  }
}


// ================================================================
//  READ LINE
//  Maps each sensor's raw ADC value to a normalised 0–1000 range
//  using its individual calibration min/max. Values above 500
//  are considered "on line". Updates the onLine flag.
// ================================================================
void readLine() {

  onLine = 0;

  for (int i = 0; i < numSensors; i++) {

    // Map to 0–1000 range, direction depending on line colour
    if (isBlackLine) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500) ? 1 : 0;

    if (sensorArray[i]) onLine = 1;
  }
}


// ================================================================
//  MOTOR CONTROL
//  Accepts signed speed (-255 to +255).
//  Positive = forward, negative = reverse, zero = coast.
//  Coast is used instead of hard brake to reduce mechanical
//  stress and avoid jerky transitions at high speed.
// ================================================================
void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, spd);
  } else if (spd < 0) {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(spd));
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, spd);
  } else if (spd < 0) {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(spd));
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
