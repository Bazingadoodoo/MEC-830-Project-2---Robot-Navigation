// encoder---------------------------------
#include <TimerOne.h>
#define leftEncoderPin 2
#define rightEncoderPin 3
float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// servo motor-----------------------------
#include <Servo.h>
#define servoPin 6
Servo myservo;

// IR receiver-----------------------------
#include <IRremote.hpp>
#define irPin 7
IRrecv irReceive(irPin);
decode_results irInput;

// motor driver----------------------------
#define enA 11      // right wheel
#define enB 10      // left wheel
#define in1 12      // forward (right)
#define in2 13      // backward (right)
#define in3 8       // backward (left)
#define in4 9       // forward (left)

void ISR_Left(){
  leftCounter++;
}

void ISR_Right(){
  rightCounter++;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),ISR_Left,RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin),ISR_Right,RISING);
  irReceive.enableIRIn(); 
}

void loop() {
  if (irReceive.decode(&irInput)){
    int irReading = irInput.value;
    switch(irReading){
      case 6375:
        DriveForward(100,1000);
        break;
      case 19125:
        DriveBackward(100,1000);
        break;
      case 4335:
        TurnLeft(100,1000);
        break;
      case 23205:
        TurnRight(100,1000);
        break;
      case 14535:
        TurnOff();
        break;
    }
      irReceive.resume();
  }
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
