#include <TimerOne.h>
#include <Servo.h>

#define leftEncoderPin 2
#define rightEncoderPin 3

// servo motor-----------------------------
#define servoPin 3

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// motor driver----------------------------
#define enA 10      // right wheel
#define enB 11      // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 8       // backward
#define in4 9       // forward

Servo myservo;

float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;

void setup() {
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),ISR_Left,RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin),ISR_right,RISING);
}

void loop() {
  
}

void TurnRight(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void TurnLeft(int motorSpeed, double delayTime){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void DriveForward(int motorSpeed, double delayTime){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void DriveBackward(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void TurnOff(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void ISR_Left{
  leftCounter++;
}


void ISR_Right{
  rightCounter++;
}
