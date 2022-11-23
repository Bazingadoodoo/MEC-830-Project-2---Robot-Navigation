//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;

// encoder---------------------------------
#define leftEncoderPin 2
#define rightEncoderPin 3
float diskSlot = 20;
int leftCounter = 0;
int rightCounter = 0;
int Right = 0;
int Left = 0;

// IR receiver-----------------------------
#include <IRremote.hpp>
#define irPin 11
IRrecv irReceive(irPin);
decode_results irInput;

// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // forward
#define in4 8       // backward
int maxSpeed = 255;
int minSpeed = 0;
int maxSpeed2 = 77;
int minSpeed2 = 0;

// proportional control--------------------
float kp = 15;
float kp2 = 0.05;                          //0.3 for target = 70
int pControlSpeedR;
int pControlSpeedL;
int initialSpeed;
float runTime;
float targetAngle;
float carOrientation = 0;
float carOrientation2 = 0;
float currentAngle;

void setup()                                                            // Setup
{
  Serial.begin(9600);
  irReceive.enableIRIn();
  Serial.println();
  Serial.println("Calibrating IMU");
  if (!bno.begin())
  {
    Serial.print("no imu sensor detected");
    while (1);
  }
  delay(2000);
  bno.setExtCrystalUse(true);
  Serial.println("Done Calibrating");
  Serial.println("Starting...");
  sensors_event_t event;
  bno.getEvent(&event);
  angle_offset = event.orientation.x;
  Serial.print("Offset: ");
  Serial.println(angle_offset);
}

void loop()                                                             // Main Loop
{
  if (irReceive.decode(&irInput))
  {
    int irReading = irInput.value;
    switch (irReading)
    {
      case 6375:                // button 2
        initialSpeed = 120;
        targetAngle = 0;
        runTime = 5000;
        DriveForward();
        break;
      case 19125:               // button 8
        initialSpeed = 120;
        targetAngle = 0;
        runTime = 5000;
        DriveBackward();
        break;
      case 4335:                // button 4
        initialSpeed = 85;
        targetAngle = carOrientation2 - 90;
        //RotateCCW ();
        break;
      case 23205:               // button 6
        initialSpeed = 70;
        targetAngle = 90;
        RotateCW();
        break;
      case 14535:               // button 5
        TurnOff();
        break;
      case 12495:               // button 1
        break;
      case 31365:               // button 3
        break;
      case -1:                  // holding button
        break;
    }
    irReceive.resume();
    //Serial.println(irReading);
  }
}

void TurnOff ()                                                         // Motor Off
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

float measure_angle(void)                                               // IMU Angle Measurement
{
  float z_angle;
  sensors_event_t event;
  bno.getEvent(&event);
  z_angle = event.orientation.x;
  //Serial.println(z_angle);
  return z_angle;
}

void SpeedControlF ()                                                   // Speed Control Forward
{
  currentAngle = measure_angle();
  if (targetAngle == 0)
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  //Serial.println(currentAngle);
  pControlSpeedR = initialSpeed + (currentAngle - targetAngle) * kp;
  if (pControlSpeedR > maxSpeed)
  {
    pControlSpeedR = maxSpeed;
  }
  if (pControlSpeedR < minSpeed)
  {
    pControlSpeedR = minSpeed;
  }

  pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp;
  if (pControlSpeedL > maxSpeed)
  {
    pControlSpeedL = maxSpeed;
  }
  if (pControlSpeedL < minSpeed)
  {
    pControlSpeedL = minSpeed;
  }
}

void SpeedControlB ()                                                   // Speed Control Backward
{
  currentAngle = measure_angle();
  if (targetAngle == 0)
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  //Serial.println(currentAngle);
  pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp;
  if (pControlSpeedR > maxSpeed)
  {
    pControlSpeedR = maxSpeed;
  }
  if (pControlSpeedR < minSpeed)
  {
    pControlSpeedR = minSpeed;
  }

  pControlSpeedL = initialSpeed + (currentAngle - targetAngle) * kp;
  if (pControlSpeedL > maxSpeed)
  {
    pControlSpeedL = maxSpeed;
  }
  if (pControlSpeedL < minSpeed)
  {
    pControlSpeedL = minSpeed;
  }
}

void DriveForward ()                                                    // Drive Forward
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  float previousTime = millis();
  while (1)
  {
    float currentTime = millis();
    SpeedControlF();
    analogWrite(enA, pControlSpeedR);
    analogWrite(enB, pControlSpeedL);
    if ((currentTime - previousTime) > runTime)
    {
      TurnOff();
      break;
    }
  }
}

void DriveBackward ()                                                   // Drive Backward
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  float previousTime = millis();
  while (1)
  {
    float currentTime = millis();
    SpeedControlB();
    analogWrite(enA, pControlSpeedR);
    analogWrite(enB, pControlSpeedL);
    if ((currentTime - previousTime) > runTime)
    {
      TurnOff();
      break;
    }
  }
}

void SpeedControlRotation ()                                            // Speed Control Rotation
{
  Serial.println("received");
  currentAngle = measure_angle();
  Serial.print("current: ");
  Serial.println(currentAngle);
  if ((targetAngle == 0) | (targetAngle == 90))
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp2;
  if (pControlSpeedR > maxSpeed2)
  {
    pControlSpeedR = maxSpeed2;
  }
  if (pControlSpeedR < minSpeed2)
  {
    pControlSpeedR = minSpeed2;

  }  

  pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp2;
  if (pControlSpeedL > maxSpeed2)
  {
    pControlSpeedL = maxSpeed2;
  }
  if (pControlSpeedL < minSpeed2)
  {
    pControlSpeedL = minSpeed2;
  }
  Serial.print("left speed: ");
  Serial.print(pControlSpeedL);
  Serial.print("  //  ");
  Serial.print("Right speed: ");
  Serial.println(pControlSpeedR);
}

void RotateCW ()                                                        // Rotate Clockwise
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  currentAngle = measure_angle();
  if ((targetAngle == 0) | (targetAngle == 90))
  {
    if (currentAngle > 180)
    {
      currentAngle -= 360;
    }
  }
  //Serial.print("current angle: ");
  //Serial.println(currentAngle);
  while (1)
  {
    Serial.print("error: ");
    Serial.println(abs(currentAngle - targetAngle));
    SpeedControlRotation();
    analogWrite(enA, pControlSpeedR);
    analogWrite(enB, pControlSpeedL);
    currentAngle = measure_angle();
  
    if (abs(currentAngle - targetAngle) < 2)
    {
      delay(100);
      if (abs(currentAngle - targetAngle) < 2)
      {
        TurnOff();
        break;
      }
    }
  }
}
