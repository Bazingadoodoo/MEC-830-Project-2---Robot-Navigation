//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Smoothed.h>
#include <Ewma.h>
#include <PID_v2.h>

Smoothed <float> average_rot_speedL;
Smoothed <float> average_rot_speedR;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
double angle;
double angleR;

//path checkpoints
bool start = 1;
bool checkpoint_1 =0;
bool checkpoint_2 =0;
bool checkpoint_3 = 0;
bool checkpoint_4 = 0;
bool rotateR = 1;

// ultrasonic sensor-----------------------
#define trigPin 4
#define echoPin 5
float distance; 

//Time variables
float previousTimeR = 0;
float currentTimeR = 0;
float previousTimeL = 0;
float currentTimeL = 0;
float loopTimeCurrent =0;
float loopTimePrevious =0;


// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

//Drive functions
float set_speed;

// PID Controller
double Kp_A = 8;
double Ki_A = 0.3;
double Kd_A = 0.4;
//8, 0.3, 0.4

double Kp_R = 2;
double Ki_R = 0.5;
double Kd_R = 0;
//2, 0.9, 0.0

double setpoint_rotate = 90.0;
double setpoint_drive = 1.0;
double output_rotate;
double output_drive;

//Proportional Control
int pControlSpeedR;
int pControlSpeedL;
float kp = 15.0;
int initialSpeed;
float targetAngle =0;
int maxSpeed = 255;
int minSpeed = 0;



PID myPID_drive(&angle,&output_drive,&setpoint_drive,Kp_A,Ki_A,Kd_A,DIRECT);
PID myPID_rotate(&angleR,&output_rotate,&setpoint_rotate,Kp_R,Ki_R,Kd_R,DIRECT);

void stop_car(void){
  setMotorL(0);
  setMotorR(0);
}

void drive_straight(double set_speed){
  myPID_drive.Compute();
  double rightSpeed = set_speed - output_drive;
  double leftSpeed = set_speed + output_drive;

  if (rightSpeed> 255){
    rightSpeed = 255;
  }
  else if (rightSpeed<0){
    rightSpeed = 0;
  }
  
  if (leftSpeed>255){
    leftSpeed = 255;
  }
  else if (leftSpeed<0){
    leftSpeed = 0;
  }

  setMotorR(rightSpeed);
  setMotorL(leftSpeed);
}



void rotate(){
  delay(1000);
  while (1){
    angleR = measure_angle();
    float error = abs(((angleR - setpoint_rotate)/(90.0))*100);
    myPID_rotate.Compute();
    double rightRotate = -output_rotate;
    double leftRotate = output_rotate;

    setMotorR(rightRotate);
    setMotorL(leftRotate);
    if (error<1)
      {
        stop_car();
        break;
      }
    }
  }


void setMotorR(int pwmR) {

  if (pwmR>0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmR<0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwmR ==0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(enA,abs(pwmR));
}

void setMotorL(int pwmL) {

  if (pwmL>0)
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if (pwmL<0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (pwmL ==0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  analogWrite(enB,abs(pwmL));
}


double measure_angle(void)
{
  double z_angle;// to determine absolute orientation 
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  z_angle = event.orientation.x; //get z-axis rotation angle

  if (z_angle > 180){
    z_angle = z_angle - 360;
  }

  return z_angle;
}

float obstacle_detection(void)
{
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  float obstacle_distance = (duration * 0.034 / 2.0) * 1.0227 + 0.0031;
  return obstacle_distance;
}

float avoid(float previousTime, float interval){
  float currentTime = millis();
  while ((currentTime - previousTime) <interval){
    currentTime = millis();
    drive_straight(120);
  }
}

float avoidance_path(int leftMotor, int rightMotor, float timeDelay){
  setMotorL(leftMotor);
  setMotorR(rightMotor);
  delay(timeDelay);
}

void setup() {
  myPID_drive.SetMode(MANUAL); //initalize PID Controller
  myPID_drive.SetSampleTime(10);
  myPID_drive.SetOutputLimits(-200, 200);

  myPID_rotate.SetMode(MANUAL); //initalize PID Controller
  myPID_rotate.SetSampleTime(10);
  myPID_rotate.SetOutputLimits(-110, 110);

  //Initalize Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin,INPUT);

  Serial.begin(9600);
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
}


void loop() {
  float loopTime = millis();
  angle = measure_angle();
  if (start){
    myPID_drive.SetMode(AUTOMATIC);
    start = 0;
    checkpoint_1 = 1;
  }

  if (checkpoint_1){
    distance = obstacle_detection();
    if (distance <= 30){
      myPID_drive.SetMode(MANUAL);
      avoidance_path(180,60,600);
      avoidance_path(60,140,500);
      myPID_drive.SetMode(AUTOMATIC);
      loopTime = millis();
      avoid(loopTime,500);
      avoidance_path(60,140,500);
      avoidance_path(180,60,600);
      myPID_drive.SetMode(AUTOMATIC);
      avoid(loopTime,2000);
      stop_car();
      checkpoint_1=0;
    }
    else
    { 
    drive_straight(110);
    }
  }
  

  // if (checkpoint_2){
  //   Serial.println("Checkpoint 2");
  //   avoid(loopTime,1500);
  //   checkpoint_2 = 0;
  //   checkpoint_3 = 1;
  //   setpoint_drive = -45.0;
  // }

  // if (checkpoint_3){
  //   Serial.println("Checkpoint 3");
  //   avoid(loopTime,1500);
  //   setpoint_drive = 0.0;
  //   checkpoint_3 = 0;
  //   checkpoint_4 = 1;
  // }

  // if (checkpoint_4){
  //   Serial.println("Checkpoint4");
  //   avoid(loopTime,1000);
  //   checkpoint_4 =0;
  //   stop_car();
  // }

}
