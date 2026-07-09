


#include <Wire.h>
#include <Servo.h>

#define MPU 0x68

// ---------------- RC INPUT ----------------
const int ch1Pin = 15;   // Roll
const int ch2Pin = 16;   // Pitch
const int ch3Pin = 17;   // Throttle
const int ch4Pin = 20;   // Yaw

// ---------------- MOTOR PINS ----------------
const int m1Pin = 3;
const int m2Pin = 4;
const int m3Pin = 5;
const int m4Pin = 6;

// ---------------- ESC ----------------
Servo motor1, motor2, motor3, motor4;

// ---------------- RAW SENSOR ----------------
int16_t ax, ay, az, gx, gy, gz;

// ---------------- ANGLES ----------------
float imuRoll = 0;
float imuPitch = 0;

// ---------------- CONTROLLER ANGLES ----------------
float setRoll = 0;
float setPitch = 0;

// ---------------- PID VALUES ----------------
float Kp = 2.0;
float Ki = 0.02;
float Kd = 0.8;

// Roll PID
float errorRoll = 0;
float prevRoll = 0;
float integralRoll = 0;
float pidRoll = 0;

// Pitch PID
float errorPitch = 0;
float prevPitch = 0;
float integralPitch = 0;
float pidPitch = 0;

// ---------------- TIME ----------------
unsigned long prevTime;
float dt;

// ---------------- MOTOR VALUES ----------------
int m1, m2, m3, m4;

// ------------------------------------------------
// FLOAT MAP
// ------------------------------------------------
float mapFloat(float x, float in_min, float in_max,
               float out_min, float out_max)
{
  return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

// ------------------------------------------------
// MPU READ
// ------------------------------------------------
void readMPU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  ax = Wire.read()<<8 | Wire.read();
  ay = Wire.read()<<8 | Wire.read();
  az = Wire.read()<<8 | Wire.read();

  Wire.read(); Wire.read();   // temp skip

  gx = Wire.read()<<8 | Wire.read();
  gy = Wire.read()<<8 | Wire.read();
  gz = Wire.read()<<8 | Wire.read();
}

// ------------------------------------------------
// SETUP
// ------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);
  pinMode(ch4Pin, INPUT);

  // Wake MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // ESC attach
  motor1.attach(m1Pin);
  motor2.attach(m2Pin);
  motor3.attach(m3Pin);
  motor4.attach(m4Pin);

  prevTime = micros();
}

// ------------------------------------------------
// LOOP
// ------------------------------------------------
void loop()
{
  // ---------- TIME ----------
  unsigned long now = micros();
  dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  // ---------- READ IMU ----------
  readMPU();

  imuRoll  = atan2(ay, az) * 180 / PI;
  imuPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;

  // ---------- READ CONTROLLER ----------
  int ch1 = pulseIn(ch1Pin, HIGH, 25000);
  int ch2 = pulseIn(ch2Pin, HIGH, 25000);
  int ch3 = pulseIn(ch3Pin, HIGH, 25000);

  if(ch1 == 0) ch1 = 1500;
  if(ch2 == 0) ch2 = 1500;
  if(ch3 == 0) ch3 = 1000;

  setRoll  = mapFloat(ch1,1000,2000,-45,45);
  setPitch = mapFloat(ch2,1000,2000,-45,45);

  // ==================================================
  // ERROR COMPARISON
  // ==================================================
  errorRoll  = setRoll  - imuRoll;
  errorPitch = setPitch - imuPitch;

  // ==================================================
  // PID ROLL
  // ==================================================
  integralRoll += errorRoll * dt;
  float dRoll = (errorRoll - prevRoll)/dt;

  pidRoll = Kp*errorRoll + Ki*integralRoll + Kd*dRoll;

  prevRoll = errorRoll;

  // ==================================================
  // PID PITCH
  // ==================================================
  integralPitch += errorPitch * dt;
  float dPitch = (errorPitch - prevPitch)/dt;

  pidPitch = Kp*errorPitch + Ki*integralPitch + Kd*dPitch;

  prevPitch = errorPitch;

  // ==================================================
  // MOTOR MIXING
  // ==================================================
  int throttle = ch3;

  m1 = throttle + pidRoll - pidPitch;
  m2 = throttle - pidRoll - pidPitch;
  m3 = throttle - pidRoll + pidPitch;
  m4 = throttle + pidRoll + pidPitch;

  m1 = constrain(m1,1000,2000);
  m2 = constrain(m2,1000,2000);
  m3 = constrain(m3,1000,2000);
  m4 = constrain(m4,1000,2000);

  // ==================================================
  // SEND TO ESC
  // ==================================================
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);

  // ==================================================
  // LIVE SERIAL DATA
  // ==================================================
  Serial.print("SetR:");
  Serial.print(setRoll);

  Serial.print(" IMUR:");
  Serial.print(imuRoll);

  Serial.print(" ErrR:");
  Serial.print(errorRoll);

  Serial.print(" PIDR:");
  Serial.print(pidRoll);

  Serial.print(" | M1:");
  Serial.print(m1);

  Serial.print(" M2:");
  Serial.print(m2);

  Serial.print(" M3:");
  Serial.print(m3);

  Serial.print(" M4:");
  Serial.println(m4);

  delay(5);
}