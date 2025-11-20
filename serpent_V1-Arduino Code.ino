#include <Arduino.h>
#include <QTRSensors.h>


#define NUM_SENSORS 8

// Motor pins (L298N)
#define ENA 13
#define IN1 12
#define IN2 14
#define ENB 27
#define IN3 26
#define IN4 25

//QTR sensor pins (left → right)
uint8_t sensorPins[NUM_SENSORS] = {33,32,35,34,39,4,36,15};

// PID constants 
float Kp = 0.5;
float Ki = 0.0;
float Kd = 3.2;


int baseSpeed = 90;
int lastError = 0;
float integral = 0;
int lastDir = 1; 


QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void setMotorLeft(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, abs(speed));
}

void setMotorRight(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, abs(speed));
}

void setup() {
  Serial.begin(115200);

  //Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);

  // calibration
  Serial.println("Starting QTR sensor calibration...");
  delay(1000);
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    if (i % 50 == 0) Serial.println("Calibrating...");
    delay(5);
  }
  Serial.println("Calibration complete!");

  // Display calibration results
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": min=");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(", max=");
    Serial.println(qtr.calibrationOn.maximum[i]);
  }

  Serial.println("\nLine following will start in 3 seconds...");
  delay(3000);
}

void loop() {
  unsigned int position = qtr.readLineBlack(sensorValues);
  int error = position - 3500; // center = 3500

  // PID computation
  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

 
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);


  if (position == 0 || position == 7000) {
    int nudge = 80;
    if (lastDir < 0) { 
      leftSpeed = baseSpeed - nudge;
      rightSpeed = baseSpeed + nudge;
    } else { 
      leftSpeed = baseSpeed + nudge;
      rightSpeed = baseSpeed - nudge;
    }
  }

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);

  lastDir = (error > 0) ? 1 : (error < 0 ? -1 : lastDir);


  Serial.print("Pos: "); Serial.print(position);
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | Corr: "); Serial.print(correction);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);

 
}

