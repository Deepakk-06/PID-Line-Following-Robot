/*
 * LINE FOLLOWER ROBOT — OPTIMIZED BUILD
 * Kp=0.13 | Kd=0.40 | Ki=0.0005 | Speed=220
 * Anti-windup | Bidirectional cal | Coast stop
 */

// ADC fast read macros
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// TB6612FNG pin definitions
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10

// Line config
bool isBlackLine = 1;
unsigned int lineThickness = 15;
unsigned int numSensors = 7;

// PID — TUNED
float Kp = 0.13;
float Kd = 0.40;
float Ki = 0.0005;

// Speed
int lfSpeed = 220;
int currentSpeed = 60;

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp, activeSensors;
int sensorWeight[7] = { 4, 2, 1, 0, -1, -2, -4 };
int onLine = 1;
int minValues[7], maxValues[7], threshold[7];
int sensorValue[7], sensorArray[7];

void setup() {
  sbi(ADCSRA, ADPS2); cbi(ADCSRA, ADPS1); cbi(ADCSRA, ADPS0);
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT); digitalWrite(5, HIGH);
  lineThickness = constrain(lineThickness, 10, 35);
  if (numSensors == 5) {
    sensorWeight[1] = 4; sensorWeight[5] = -4;
  }
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);
  I = 0; previousError = 0; currentSpeed = 60;
  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
      I = 0;
      if (error > 0) { motor1run(-80); motor2run(lfSpeed); }
      else           { motor1run(lfSpeed); motor2run(-80); }
    }
  }
}

void linefollow() {
  error = 0; activeSensors = 0;
  int s = (numSensors == 5) ? 1 : 0;
  int e = (numSensors == 5) ? 6 : 7;
  for (int i = s; i < e; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }
  if (activeSensors > 0) error = error / activeSensors;
  P = error;
  I = constrain(I + error, -500, 500);
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
  lsp = constrain(currentSpeed - PIDvalue, 0, 255);
  rsp = constrain(currentSpeed + PIDvalue, 0, 255);
  motor1run(lsp); motor2run(rsp);
}

void calibrate() {
  Serial.println("Calibrating...");
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  for (int j = 0; j < 5000; j++) {
    motor1run(60); motor2run(-60);
    for (int i = 0; i < 7; i++) {
      int v = analogRead(i);
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
  }
  for (int j = 0; j < 5000; j++) {
    motor1run(-60); motor2run(60);
    for (int i = 0; i < 7; i++) {
      int v = analogRead(i);
      if (v < minValues[i]) minValues[i] = v;
      if (v > maxValues[i]) maxValues[i] = v;
    }
  }
  motor1run(0); motor2run(0);
  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]); Serial.print(" ");
  }
  Serial.println("\nDone.");
}

void readLine() {
  onLine = 0;
  int s = (numSensors == 5) ? 1 : 0;
  int e = (numSensors == 5) ? 6 : 7;
  for (int i = s; i < e; i++) {
    if (isBlackLine)
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    else
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = (sensorValue[i] > 500) ? 1 : 0;
    if (sensorArray[i]) onLine = 1;
  }
}

void motor1run(int spd) {
  spd = constrain(spd, -255, 255);
  if      (spd > 0) { digitalWrite(AIN1,1); digitalWrite(AIN2,0); analogWrite(PWMA, spd); }
  else if (spd < 0) { digitalWrite(AIN1,0); digitalWrite(AIN2,1); analogWrite(PWMA, abs(spd)); }
  else              { digitalWrite(AIN1,0); digitalWrite(AIN2,0); analogWrite(PWMA, 0); }
}

void motor2run(int spd) {
  spd = constrain(spd, -255, 255);
  if      (spd > 0) { digitalWrite(BIN1,1); digitalWrite(BIN2,0); analogWrite(PWMB, spd); }
  else if (spd < 0) { digitalWrite(BIN1,0); digitalWrite(BIN2,1); analogWrite(PWMB, abs(spd)); }
  else              { digitalWrite(BIN1,0); digitalWrite(BIN2,0); analogWrite(PWMB, 0); }
}
