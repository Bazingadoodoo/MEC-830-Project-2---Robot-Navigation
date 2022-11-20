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
/*#include <Servo.h>
#define servoPin 6
Servo myservo;*/

// IR receiver-----------------------------
#include <IRremote.hpp>
#define irPin 11
IRrecv irReceive(irPin);
decode_results irInput;

// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6      // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

void TurnRight(int motorSpeedL, int motorSpeedR, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void TurnLeft(int motorSpeedL, int motorSpeedR, double delayTime){
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void DriveForward(int motorSpeedL, int motorSpeedR, double delayTime){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void DriveBackward(int motorSpeedL, int motorSpeedR, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void TurnOff(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

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
          DriveForward(100,90,1);
          break;
        case 19125:
          DriveBackward(100,90,1);
          break;
        case 4335:
          TurnLeft(100,70,1);
          break;
        case 23205:
          TurnRight(100,70,1);
          break;
        case 14535:
          TurnOff();
          break;
        case 12495:
          TurnLeft(0,65,1);
          break;
        case 31365:
          TurnRight(90,0,1);
          break;
    }
    irReading = 0;
    irReceive.resume();
  }
}
