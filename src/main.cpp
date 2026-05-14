#include <Arduino.h>
#include <ESP32Servo.h>

#define Brush1_A 5
#define Brush1_B 18
#define Brush2_A 19
#define Brush2_B 21
#define BTN_PIN 23
#define DRIVER_EN 22

bool systemStarted = false;
bool lastBtnState = HIGH;

/* ---------- Motors ---------- */
#define F1_A 25
#define F1_B 26
#define F2_A 27
#define F2_B 32

#define B1_A 33
#define B1_B 14
#define B2_A 12
#define B2_B 13

#define CH_B1A 4
#define CH_B1B 5
#define CH_B2A 6
#define CH_B2B 7

#define MOTOR_SPEED 150
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

/* ---------- HC-SR04 ---------- */
#define TRIG 4
#define ECHO 17

#define SOUND_SPEED 0.0343
#define LIMIT_CM 40.0

#define SERVO_PIN 15
#define SERVO_CENTER 90
#define SERVO_LEFT   150
#define SERVO_RIGHT  30

Servo scanServo;

void brushesForward() {
  digitalWrite(Brush1_A, HIGH);
  digitalWrite(Brush1_B, LOW);
  digitalWrite(Brush2_A, HIGH);
  digitalWrite(Brush2_B, LOW);
}

void brushesStop() {
  digitalWrite(Brush1_A, LOW);
  digitalWrite(Brush1_B, LOW);
  digitalWrite(Brush2_A, LOW);
  digitalWrite(Brush2_B, LOW);
}

void motorForward(int pinA, int pinB, int speed2 = 255) {
  if (pinA == B1_A && pinB == B1_B) {
    ledcWrite(CH_B1A, speed2);
    ledcWrite(CH_B1B, 0);
  }
  else if (pinA == B2_A && pinB == B2_B) {
    ledcWrite(CH_B2A, speed2);
    ledcWrite(CH_B2B, 0);
  }
  else {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  }
}

void motorBackward(int pinA, int pinB) {
  if (pinA == B1_A && pinB == B1_B) {
    ledcWrite(CH_B1A, 0);
    ledcWrite(CH_B1B, MOTOR_SPEED);
  }
  else if (pinA == B2_A && pinB == B2_B) {
    ledcWrite(CH_B2A, 0);
    ledcWrite(CH_B2B, MOTOR_SPEED);
  }
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
}

void motorStop(int pinA, int pinB) {
  if (pinA == B1_A && pinB == B1_B) {
    ledcWrite(CH_B1A, 0);
    ledcWrite(CH_B1B, 0);
  }
  else if (pinA == B2_A && pinB == B2_B) {
    ledcWrite(CH_B2A, 0);
    ledcWrite(CH_B2B, 0);
  }
  else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void moveForward() {
  motorForward(B1_A, B1_B, MOTOR_SPEED);
  motorForward(B2_A, B2_B, MOTOR_SPEED);

  brushesForward();
}

void moveBackward() {
  motorBackward(F1_A, F1_B);
  motorBackward(F2_A, F2_B);
  motorBackward(B1_A, B1_B);
  motorBackward(B2_A, B2_B);

  brushesStop();
}

void turnLeft() {
  motorForward(F1_A, F1_B);
  motorForward(B2_A, B2_B);
  motorStop(F2_A, F2_B);
  motorStop(B1_A, B1_B);

  brushesStop();
}

void turnRight() {
  motorForward(F2_A, F2_B);
  motorForward(B1_A, B1_B);
  motorStop(F1_A, F1_B);
  motorStop(B2_A, B2_B);

  brushesStop();
}

void moveStopAll() {
  motorStop(F1_A, F1_B);
  motorStop(F2_A, F2_B);
  motorStop(B1_A, B1_B);
  motorStop(B2_A, B2_B);

  brushesStop();
}

void shutdownAll() {
  moveStopAll();
  digitalWrite(DRIVER_EN, LOW);
}

float readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH, 25000);

  if (duration == 0) return -1;

  return (duration * SOUND_SPEED) / 2.0;
}

float readDistanceStable(uint8_t samples = 3) {
  float sum = 0;
  int ok = 0;

  for (uint8_t i = 0; i < samples; i++) {
    float d = readDistance();

    if (d > 0) {
      sum += d;
      ok++;
    }

    delay(40);
  }

  if (ok == 0) return -1;

  return sum / ok;
}

void setup() {
  Serial.begin(115200);
  randomSeed((uint32_t)esp_random());

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(DRIVER_EN, OUTPUT);

  digitalWrite(DRIVER_EN, LOW);

  pinMode(Brush1_A, OUTPUT);
  pinMode(Brush1_B, OUTPUT);
  pinMode(Brush2_A, OUTPUT);
  pinMode(Brush2_B, OUTPUT);

  pinMode(F1_A, OUTPUT);
  pinMode(F1_B, OUTPUT);
  pinMode(F2_A, OUTPUT);
  pinMode(F2_B, OUTPUT);

  ledcSetup(CH_B1A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_B1B, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_B2A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_B2B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(B1_A, CH_B1A);
  ledcAttachPin(B1_B, CH_B1B);
  ledcAttachPin(B2_A, CH_B2A);
  ledcAttachPin(B2_B, CH_B2B);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  scanServo.setPeriodHertz(35);
  scanServo.attach(SERVO_PIN, 100, 1500);
  scanServo.write(SERVO_CENTER);
  delay(500);

  shutdownAll();
}

void loop() {
  bool btnState = digitalRead(BTN_PIN);

  if (lastBtnState == HIGH && btnState == LOW) {
    systemStarted = !systemStarted;
    delay(200);
  }

  lastBtnState = btnState;

  if (!systemStarted) {
    shutdownAll();
    return;
  }

  digitalWrite(DRIVER_EN, HIGH);

  scanServo.write(SERVO_CENTER);

  float distance = readDistanceStable(3);

  Serial.print("Distance front: ");
  Serial.println(distance);

  if (distance > LIMIT_CM || distance < 0) {
    moveForward();
    delay(80);
    return;
  }

  moveStopAll();
  delay(100);

  moveBackward();
  delay(400);

  moveStopAll();
  delay(200);

  scanServo.write(SERVO_RIGHT);
  delay(500);
  float rightD = readDistanceStable(3);
  delay(100);

  scanServo.write(SERVO_CENTER);
  delay(200);

  scanServo.write(SERVO_LEFT);
  delay(500);
  float leftD = readDistanceStable(3);
  delay(100);

  scanServo.write(SERVO_CENTER);
  delay(200);

  if (leftD < 0) leftD = 0;
  if (rightD < 0) rightD = 0;

  if (rightD >= leftD) {
    turnRight();
  } else {
    turnLeft();
  }

  delay(500);

  moveStopAll();
  delay(200);
}
