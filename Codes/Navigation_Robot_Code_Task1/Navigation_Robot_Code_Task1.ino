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

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// servo motor-----------------------------
#include <Servo.h>
#define servoPin 6

// IR receiver-----------------------------
#include <IRremote.hpp>
#define irPin 7
IRrecv irReceive(irPin);
decode_results irInput;

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
  Serial.println("Calibrating IMU");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the imu */
    Serial.print("no imu sensor detected");
    while(1);
    
    delay(2000);
    bno.setExtCrystalUse(true);
    Serial.println("Done Calibrating");
    Serial.println("Starting...");
    sensors_event_t event; 
    bno.getEvent(&event);
    angle_offset = event.orientation.x; //get z-axis rotation angle
    Serial.println(angle_offset);
  }
}

void loop() {
  angle = measure_angle();
  Serial.print("left counter: ");
  Serial.print(leftCounter);
  Serial.print("  //  ");
  Serial.print("right counter: ");
  Serial.println(rightCounter);
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
