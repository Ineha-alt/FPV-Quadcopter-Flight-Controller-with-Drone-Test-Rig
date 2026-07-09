#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;

int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float Yaw = 0;   // ✅ NEW

uint32_t LoopTimer;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;

float Kalman1DOutput[]={0,0};

// ---------------- KALMAN ----------------
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput; // state equation update with 4000 us
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4; // covariance

  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3); // gain matrix

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState); // state update
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty; // covariance update

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ---------------- IMU ----------------
void gyro_signals(void) {

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();    

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);

  int16_t GyroX = Wire.read()<<8 | Wire.read();
  int16_t GyroY = Wire.read()<<8 | Wire.read();
  int16_t GyroZ = Wire.read()<<8 | Wire.read();

  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll  = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 57.3;
  AnglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 57.3;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(57600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();

  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;
    delay(1);
  }

  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;

  LoopTimer = micros();
}

// ---------------- LOOP ----------------
void loop() {

  gyro_signals();

  // Remove bias
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  // Kalman (roll + pitch)
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // ---------------- YAW (GYRO INTEGRATION) ----------------
  Yaw += RateYaw * 0.004;   // dt = 4ms

  // ---------------- OUTPUT ----------------
  // Serial.print("Roll: ");
  Serial.print(KalmanAngleRoll);



  // Serial.print(" Pitch: ");
  Serial.print("  ");
  Serial.print(KalmanAnglePitch);

  // Serial.print(" Yaw: ");
  Serial.print("  ");
  Serial.println(Yaw);

  while (micros() - LoopTimer < 4000);{ // delay for 4000 us
    LoopTimer = micros();
  }
 
}