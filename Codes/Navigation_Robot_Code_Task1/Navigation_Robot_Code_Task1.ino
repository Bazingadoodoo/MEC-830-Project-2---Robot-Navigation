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
        DriveForward(255,1);
        break;
      case 19125:
        DriveBackward(255,1);
        break;
      case 4335:
        TurnLeft(100,1);
        break;
      case 23205:
        TurnRight(100,1);
        break;
      case 14535:
        TurnOff();
        break;
    }
    irReading = 0;
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
void ISR_Right(){
  if (Right){
    rightCounter++;
  }
  else{
    rightCounter--;
  }
}

void RotateRight(int motorSpeedL, int motorSpeedR, int startAngle){
  double Zrot = measure_angle();
  double Zrot_mapped = Zrot + 180;
  if (Zrot_mapped >= 360){
    Zrot_mapped = Zrot_mapped-360;
  }
  if (Zrot-startAngle<90){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, motorSpeedR);
    analogWrite(enB, motorSpeedL);
    delay(20);
  }
}

void TurnRight(int motorSpeedL, int motorSpeedR, double delayTime){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
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
  TurnOff();
}

void DriveBackward(int motorSpeed, double delayTime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
  delay(delayTime);
  TurnOff();
}

void DriveBackward(int motorSpeedL, int motorSpeedR, double delayTime){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
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
  //Serial.println(z_angle);
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
  irReceive.enableIRIn(); 
  Serial.println("Calibrating IMU");

  /* Initialise the sensor */
  //if(!bno.begin())
  {
    /* There was a problem detecting the imu */
    /*
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
    */
  }
}

void loop() {
    /*
    double Z = measure_angle();
    double Z_mapped = Z + 180;
    if (Z_mapped >= 360){
      Z_mapped = Z_mapped-360;
    }
    */
    if (irReceive.decode(&irInput)){
      int irReading = irInput.value;
      switch(irReading){
        case 6375:                // button 2
          DriveForward(117,100,1000);
          break;  
        case 19125:               // button 8
          DriveBackward(115,95,1000);
          break;
        case 4335:                // button 4
          break;
        case 23205:               // button 6
          //RotateRight(100,71,Z_mapped);
          break;
        case 14535:               // button 5
          TurnOff();
          break;
        case 12495:               // button 1
          TurnLeft(0,100,300);
          break;
        case 31365:               // button 3
          TurnRight(110,0,300);
          break;
    }
    irReceive.resume();
    Serial.println(irReading);
  }
  /*
  Serial.print("left counter: ");
  Serial.print(leftCounter);
  Serial.print("  //  ");
  Serial.print("right counter: ");
  Serial.println(rightCounter);
  */
}
