const int ch1Pin = 15;   // Roll
const int ch2Pin = 16;   // Pitch
const int ch3Pin = 17;   // Throttle (optional)
const int ch4Pin = 20;   // Yaw

// -------- VARIABLES --------
int ch1Value = 1500;
int ch2Value = 1500;
int ch3Value = 1000;
int ch4Value = 1500;

float rollAngle, pitchAngle, yawAngle;

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);
  pinMode(ch4Pin, INPUT);
}

// -------- LOOP --------
void loop() {

  // Read PWM signal (1000–2000 µs)
  ch1Value = pulseIn(ch1Pin, HIGH, 25000);
  ch2Value = pulseIn(ch2Pin, HIGH, 25000);
  ch3Value = pulseIn(ch3Pin, HIGH, 25000);
  ch4Value = pulseIn(ch4Pin, HIGH, 25000);

  // Safety fallback
  if (ch1Value == 0) ch1Value = 1500;
  if (ch2Value == 0) ch2Value = 1500;
  if (ch4Value == 0) ch4Value = 1500;

  // Apply deadband ±10 around center (1500)
  ch1Value = applyDeadband(ch1Value, 1500, 10);
  ch2Value = applyDeadband(ch2Value, 1500, 10);
  ch4Value = applyDeadband(ch4Value, 1500, 10);

  // -------- MAPPING --------
  // Convert 1000–2000 → -45° to +45°
  rollAngle  = mapFloat(ch1Value, 1000, 2000, -45, 45);
  pitchAngle = mapFloat(ch2Value, 1000, 2000, -45, 45);
  yawAngle   = mapFloat(ch4Value, 1000, 2000, -45, 45);

  // -------- PRINT --------
  Serial.print("Roll: ");
  Serial.print(rollAngle);
  Serial.print("  Pitch: ");
  Serial.print(pitchAngle);
  Serial.print("  Yaw: ");
  Serial.println(yawAngle);

  delay(20);
}

// -------- FLOAT MAP FUNCTION --------
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// -------- DEADBAND FUNCTION --------
int applyDeadband(int value, int center, int deadband) {
  if (value > center - deadband && value < center + deadband) {
    return center;
  }
  return value;
}