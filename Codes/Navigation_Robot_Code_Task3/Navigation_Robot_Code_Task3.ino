//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;

// encoder---------------------------------
#include <TimerOne.h>
#define leftEncoderPin 2
#define rightEncoderPin 3
float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;
int Right = 0;
int Left = 0;

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5
float duration;
float obstacle_distance;

// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

void ISR_Left() {
  if (Left) {
    leftCounter++;
  }
  else {
    leftCounter--;
  }
}

void ISR_Right() {
  if (Right) {
    rightCounter++;
  }
  else {
    rightCounter--;
  }
}

void TurnRight(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 0;
  Left = 1;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void TurnLeft(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 1;
  Left = 0;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void DriveForward(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 1;
  Left = 1;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void DriveBackward(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 0;
  Left = 0;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
}

void TurnOff() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

float measure_angle(void)
{
  float z_angle;// to determine absolute orientation
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  z_angle = event.orientation.x; //get z-axis rotation angle
  Serial.println(z_angle);
  return z_angle;
}


void ISR_ObstacleDetection() {
  Timer1.detachInterrupt();
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  obstacle_distance = (duration * 0.034 / 2.0) * 1.0227 + 0.0031;
  Serial.println(obstacle_distance);
  Timer1.attachInterrupt(ISR_ObstacleDetection);
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), ISR_Left, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), ISR_Right, RISING);
  Timer1.attachInterrupt(ISR_ObstacleDetection);
}

void loop() {

}
