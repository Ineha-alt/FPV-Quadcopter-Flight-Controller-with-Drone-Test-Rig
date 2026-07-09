#define CH1 15
#define CH2 16
#define CH3 17
#define CH4 20
#define CH5 22
#define CH6 21

volatile unsigned start1 = 0, val1 = 0;
volatile unsigned start2 = 0, val2 = 0;
volatile unsigned start3 = 0, val3 = 0;
volatile unsigned start4 = 0, val4 = 0;
volatile unsigned start5 = 0, val5 = 0;
volatile unsigned start6 = 0, val6 = 0;


void isr1() {
  if (digitalRead(CH1) == HIGH)
    start1 = micros();
  else
    val1 = micros() - start1;
}


void isr2() {
  if (digitalRead(CH2) == HIGH)
    start2 = micros();
  else
    val2 = micros() - start2;
}

void isr3() {
  if (digitalRead(CH3) == HIGH)
    start3 = micros();
  else
    val3 = micros() - start3;
}


void isr4() {
  if (digitalRead(CH4) == HIGH)
    start4 = micros();
  else
    val4 = micros() - start4;
}


void isr5() {
  if (digitalRead(CH5) == HIGH)
    start5 = micros();
  else
    val5 = micros() - start5;
}


void isr6() {
  if (digitalRead(CH6) == HIGH)
    start6 = micros();
  else
    val6 = micros() - start6;
}

void setup() {
  Serial.begin(115200);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5, INPUT);
  pinMode(CH6, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH3), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4), isr4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH5), isr5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH6), isr6, CHANGE);
}

void loop() {
  Serial.print("CH1: "); Serial.print(val1);
  Serial.print(" | CH2: "); Serial.print(val2);
  Serial.print(" | CH3: "); Serial.print(val3);
  Serial.print(" | CH4: "); Serial.print(val4);
  Serial.print(" | CH5: "); Serial.print(val5);
  Serial.print(" | CH6: "); Serial.println(val6);

  delay(20);
}