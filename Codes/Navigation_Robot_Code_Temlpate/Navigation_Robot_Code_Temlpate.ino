#include <TimerOne.h>
#include <Servo.h>

// servo motor-----------------------------
#define servoPin 3

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// motor driver----------------------------
#define enA 10      
#define enB 11      
#define in1 12      
#define in2 13      
#define in3 8       
#define in4 9       

Servo myservo;

void setup() {
  myservo.attach(servoPin);
}

void loop() {

}

void TurnRight(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime)
}

void TurnLeft(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime)
}

void DriveForward(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime)
}

void DriveForward(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime)
}

void TurnOff(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
