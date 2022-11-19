
#include <Wire.h>
#include <Encoder.h>
#include <Smoothed.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v2.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
#define PI 3.14159265358979323



//Time variables
long previousTime = 0;
long currentTime = 0;

//Loop Timing
long loop_time = 0;
long previous_looptime = 0.00;
long deltaT = 0;
long setupDelay; //accounts for time for setup

//Encoder variables
volatile long currentEncoder;
volatile float oldPosition = 0;
volatile float newPosition = 0;
volatile float rot_speed;
const int Encoder_counts = 48; //define number of counts in one round of encoder

Smoothed <float> average_rot_speed;
double angle;

// Motor Variables 
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

// PID controller Variables

double Kp = 30;
double Ki = 0;
double Kd = 2.4;

//30 0 3
//40 0 5
//33 2.5 
// 17 4.6 2.9
//17 4 2.9

double setpoint = 0.0;
double angular_position;
double output;
double current;
double error;
double prevError = 0;
double integral;
double derivative;
double voltage;
double PWM;

//angle and speed adjustment
double angle_rate = 1e-6;
double angle_rate_speed = 0.1e-6; // reduce rotation


PID myPID(&angle,&output,&setpoint,Kp,Ki,Kd,DIRECT);



float measure_angle(void)
{
  float y_angle; // rotation of pendulum arm
  float z_angle;// to determine absolute orientation 
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  z_angle = event.orientation.z; //get z-axis rotation angle
  y_angle = event.orientation.y; //get y-axis rotation angle

  if (z_angle < 0)
    y_angle = -(y_angle + 90);
  else if (z_angle > 0)
    y_angle = y_angle + 90;
  else if (y_angle == 90|| y_angle == -90)
    y_angle = 0.0;
  return y_angle;
}

double adjust_setpoint(double targetAngle, double measuredAngle, double angularSpeed, long deltaTime)
{
  double angle_rate = 1e-6;
  if (measuredAngle < targetAngle)
    targetAngle += (double) angle_rate*(deltaTime);
  else
    targetAngle -= (double) angle_rate*(deltaTime);

  targetAngle -= angle_rate_speed*angularSpeed*deltaTime;

  return targetAngle;
}



void setup() {

  myPID.SetMode(AUTOMATIC); //initalize PID Controller
  myPID.SetSampleTime(10);
  myPID.SetOutputLimits(-255, 255);

  Serial.begin(9600);   
  
  //Initalize Motor Pins 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);  
  digitalWrite(ENB, HIGH);
  
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
  delay(2000);
  Serial.println("Starting...");
  setupDelay = micros();
}


void loop() {
  //loop_time = micros() - setupDelay;
  //deltaT = (double) (loop_time - previous_looptime);
  //previous_looptime = loop_time;

  angle = measure_angle();
  //setpoint = adjust_setpoint(setpoint,angle,0.0,deltaT);
  
  myPID.Compute();
  //Set Motor Direction
  if (output<0)
  {
    digitalWrite(IN1, HIGH);      
    digitalWrite(IN2, LOW); 
    digitalWrite(IN3, LOW);      
    digitalWrite(IN4, HIGH);  //go forward
  }
  else
  {
    digitalWrite(IN1, LOW);      
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);      
    digitalWrite(IN4, LOW);   //go back
  }

  //Stop if angle too large
  if (abs(angle)>27)
    output = 0;

  analogWrite(ENA, abs(output));
  analogWrite(ENB, abs(output));

  Serial.print(angle);
  Serial.print(" ");
  Serial.println(output);
}