
#include <Wire.h>


#define MPU 0x68

//RECEIVER PINS
#define CH1 15   // Roll
#define CH2 16   // Pitch
#define CH3 17   // Throttle
#define CH4 20   // Yaw

//MOTOR OUTPUT PINS 
#define M1_PIN 3
#define M2_PIN 4
#define M3_PIN 5
#define M4_PIN 6

/*************** RECEIVER ISR VARIABLES ***************/
volatile unsigned long start1 = 0, ch1Raw = 1500;
volatile unsigned long start2 = 0, ch2Raw = 1500;
volatile unsigned long start3 = 0, ch3Raw = 1000;
volatile unsigned long start4 = 0, ch4Raw = 1500;

/*************** IMU RAW ***************/
int16_t ax, ay, az, gx, gy, gz;

/*************** ANGLES ***************/
float rollAngle = 0;
float pitchAngle = 0;
float yawRate = 0;

/*************** KALMAN ***************/
float angle_roll = 0, bias_roll = 0;
float angle_pitch = 0, bias_pitch = 0;

float P_roll[2][2] = {{1,0},{0,1}};
float P_pitch[2][2] = {{1,0},{0,1}};

float Q_angle = 0.001;
float Q_bias  = 0.003;
float R_measure = 0.03;

/*************** PID ***************/
float Kp = 2.0;
float Ki = 0.02;
float Kd = 0.8;

float error_roll, prev_error_roll = 0, integral_roll = 0;
float error_pitch, prev_error_pitch = 0, integral_pitch = 0;

float pid_roll, pid_pitch;

/*************** TIME ***************/
unsigned long prevTime;
float dt;

/*************** MOTOR VALUES ***************/
int m1, m2, m3, m4;

/**************************************************
                    ISR FUNCTIONS
**************************************************/
void isr1() {
  if (digitalRead(CH1)) start1 = micros();
  else ch1Raw = micros() - start1;
}

void isr2() {
  if (digitalRead(CH2)) start2 = micros();
  else ch2Raw = micros() - start2;
}

void isr3() {
  if (digitalRead(CH3)) start3 = micros();
  else ch3Raw = micros() - start3;
}

void isr4() {
  if (digitalRead(CH4)) start4 = micros();
  else ch4Raw = micros() - start4;
}

/**************************************************
                 FLOAT MAP FUNCTION
**************************************************/
float mapFloat(float x, float in_min, float in_max,
               float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) /
         (in_max - in_min) + out_min;
}

/**************************************************
                  READ IMU
**************************************************/
void readMPU() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

/**************************************************
               KALMAN FILTER
**************************************************/
float kalman(float newAngle, float newRate, float dt,
             float &angle, float &bias, float P[2][2]) {

  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = newAngle - angle;

  angle += K0 * y;
  bias  += K1 * y;

  float P00 = P[0][0];
  float P01 = P[0][1];

  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}

/**************************************************
                     SETUP
**************************************************/
void setup() {

  Serial.begin(115200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4), isr4, CHANGE);

  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(M3_PIN, OUTPUT);
  pinMode(M4_PIN, OUTPUT);

  Wire.begin();

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  prevTime = micros();
}

/**************************************************
                     LOOP
**************************************************/
void loop() {

  /******** TIME ********/
  unsigned long now = micros();
  dt = (now - prevTime) / 1000000.0;
  prevTime = now;

  /******** READ IMU ********/
  readMPU();

  float accRoll =
      atan2(ay, az) * 180 / PI;

  float accPitch =
      atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  float gyroRoll = gx / 131.0;
  float gyroPitch = gy / 131.0;
  yawRate = gz / 131.0;

  /******** FILTER ********/
  rollAngle = kalman(accRoll, gyroRoll, dt,
                     angle_roll, bias_roll, P_roll);

  pitchAngle = kalman(accPitch, gyroPitch, dt,
                      angle_pitch, bias_pitch, P_pitch);

  /******** RECEIVER MAP ********/
  float set_roll =
      mapFloat(ch1Raw, 1000, 2000, -45, 45);

  float set_pitch =
      mapFloat(ch2Raw, 1000, 2000, -45, 45);

  int throttle = ch3Raw;

  /******** DEADBAND ********/
  if (abs(set_roll) < 2) set_roll = 0;
  if (abs(set_pitch) < 2) set_pitch = 0;

  /******** PID ROLL ********/
  error_roll = set_roll - rollAngle;

  integral_roll += error_roll * dt;
  float d_roll =
      (error_roll - prev_error_roll) / dt;

  pid_roll =
      Kp * error_roll +
      Ki * integral_roll +
      Kd * d_roll;

  prev_error_roll = error_roll;

  /******** PID PITCH ********/
  error_pitch = set_pitch - pitchAngle;

  integral_pitch += error_pitch * dt;
  float d_pitch =
      (error_pitch - prev_error_pitch) / dt;

  pid_pitch =
      Kp * error_pitch +
      Ki * integral_pitch +
      Kd * d_pitch;

  prev_error_pitch = error_pitch;

  /******** LIMIT PID ********/
  pid_roll = constrain(pid_roll, -300, 300);
  pid_pitch = constrain(pid_pitch, -300, 300);

  /******** MOTOR MIXING ********/
  m1 = throttle + pid_roll - pid_pitch;
  m2 = throttle - pid_roll - pid_pitch;
  m3 = throttle - pid_roll + pid_pitch;
  m4 = throttle + pid_roll + pid_pitch;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  /******** OUTPUT TO ESC ********/
  analogWrite(M1_PIN, map(m1,1000,2000,0,255));
  analogWrite(M2_PIN, map(m2,1000,2000,0,255));
  analogWrite(M3_PIN, map(m3,1000,2000,0,255));
  analogWrite(M4_PIN, map(m4,1000,2000,0,255));

  /******** DEBUG ********/
  Serial.print("Roll:");
  Serial.print(rollAngle);

  Serial.print(" Set:");
  Serial.print(set_roll);

  Serial.print(" M1:");
  Serial.print(m1);

  Serial.print(" M2:");
  Serial.print(m2);

  Serial.print(" M3:");
  Serial.print(m3);

  Serial.print(" M4:");
  Serial.println(m4);

  delay(5);
}