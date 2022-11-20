#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.1415926535897932384626433832795

//IMU--------------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;

// encoder---------------------------------
#define leftEncoderPin 2
#define rightEncoderPin 3
float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;
float prev_encoderR = 0;
flot prev_encoderL=0;
long current_time = 0;
long previous_time = 0;
float x = 0;
float y = 0;
float theta = 0;


//ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5   

// servo motor-----------------------------
#define servoPin 6
Servo myservo;

// IR receiver-----------------------------
#define irPin 7

// motor driver----------------------------
#define enA 10      // right wheel
#define enB 11      // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 8       // backward
#define in4 9       // forward

void ISR_Left(){
  leftCounter++;
}

void ISR_Right(){
  rightCounter++;
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

void Odometry(float offsetR, float offsetL) {	
    float R = 0.03325;

    float SR = ((rightCounter - offsetR)/20)*(2*PI*R);
    float SL = ((leftCounter - offsetL)/20)*(2*PI*R);
    
    float meanDistance = (SR+SL)/2;

    return meanDistance;
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
  }

  delay(2000);
  bno.setExtCrystalUse(true);
  Serial.println("Done Calibrating");
  Serial.println("Starting...");
  sensors_event_t event; 
  bno.getEvent(&event);
  angle_offset = event.orientation.x; //get z-axis rotation angle
  Serial.println(angle_offset);
}

void loop() {
  angle = measure_angle();
  // Serial.print("left counter: ");
  // Serial.print(leftCounter);
  // Serial.print("  //  ");
  // Serial.print("right counter: ");
  // Serial.print(rightCounter);
  // Serial.print("  //  ");
  //Serial.println(angle);
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