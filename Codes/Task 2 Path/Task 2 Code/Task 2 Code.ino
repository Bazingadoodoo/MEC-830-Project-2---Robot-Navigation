//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Smoothed.h>

Smoothed <float> average_rot_speedL;
Smoothed <float> average_rot_speedR;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;
bool start = 1; 

// encoder---------------------------------
#define leftEncoderPin 2
#define rightEncoderPin 3
float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;
int Right = 1;
int Left = 1;

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// servo motor-----------------------------
/*#include <Servo.h>
#define servoPin 6
Servo myservo;*/

//Encoder variables
volatile float currentEncoderR;
volatile float oldPositionR = 0;
volatile float newPositionR = 0;
volatile float rot_speedR;
volatile int prevEncoderR =0;
volatile int prevEncoderL =0; 

//Time variables
float previousTimeR = 0;
float currentTimeR = 0;
float previousTimeL = 0;
float currentTimeL = 0;
long currentTime;
long previousTime =0;

volatile long currentEncoderL;
volatile float oldPositionL = 0;
volatile float newPositionL = 0;
volatile float rot_speedL;

// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

// Odometry-----
float R = 0.03325;
float posX;
float posY;
float meanDistance;

void ISR_Left(){
  if (Left){
    leftCounter++;
  }
  else{
    leftCounter--;
  }
}

void ISR_Right(){
  if (Right){
    rightCounter++;
  }
  else{
    rightCounter--;
  }
}



void TurnRight(int motorSpeed, double delayTime){
  Right = 0;
  Left = 1;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void TurnLeft(int motorSpeed, double delayTime){
  Right = 1;
  Left = 0;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void DriveForward(int motorSpeed, double delayTime){
  Right = 1;
  Left = 1;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
}

void DriveBackward(int motorSpeed, double delayTime){
  Right = 0;
  Left = 0;
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
  return z_angle;
}


void Odometry(float theta) {

  currentTime = millis();

  if ((currentTime -previousTime)>200){

    float SR = (float)((rightCounter-prevEncoderR)/80.0)*(2*PI*R);
    float SL = (float)((leftCounter-prevEncoderL)/80.0)*(2*PI*R);

    prevEncoderL = leftCounter;
    prevEncoderR = rightCounter;

    meanDistance = (SL + SR) / 2;
    posX = posX + meanDistance * cos (theta);
    posY = posY + meanDistance * sin(theta);
    previousTime = currentTime;
  }

}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),ISR_Left,RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin),ISR_Right,RISING);
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
    average_rot_speedL.begin(SMOOTHED_AVERAGE,10); //prepare smoothing library 
    average_rot_speedR.begin(SMOOTHED_AVERAGE,10); //prepare smoothing library 
  }
}

void slow_acceleration(){
  for (int i =0; i<200;i++){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, i);
      analogWrite(enB, i);
      delay(50);
  }
}


void loop() {
  angle = measure_angle();
  Odometry(angle);
  if (start){
    slow_acceleration();
    start = 0;
    Serial.println(start);
  }

  // if (posX <=1){
  //   Right = 1;
  //   Left = 1;
  //   digitalWrite(in1, HIGH);
  //   digitalWrite(in2, LOW);
  //   digitalWrite(in3, LOW);
  //   digitalWrite(in4, HIGH);
  //   analogWrite(enA, 150);
  //   analogWrite(enB, 200);
  // }
  // else{
  //   analogWrite(enA, 0);
  //   analogWrite(enB, 0);
  // }
  
}
