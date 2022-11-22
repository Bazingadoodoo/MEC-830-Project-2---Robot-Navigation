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

// proportional control--------------------
float kp = 15;      
int pControlSpeedR;
int pControlSpeedL;

void setup()
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

void loop()
{
  if (irReceive.decode(&irInput))
  {
    int irReading = irInput.value;
    switch (irReading)
    {
      case 6375:                // button 2
        DriveForward(120,0);
        break;
      case 19125:               // button 8
        DriveBackward(120,0);
        break;
      case 4335:                // button 4
        break;
      case 23205:               // button 6
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

float measure_angle(void)
{
  float z_angle;
  sensors_event_t event;
  bno.getEvent(&event);
  z_angle = event.orientation.x;
  //Serial.println(z_angle);
  return z_angle;
}

void SpeedControl (int initialSpeed, float targetAngle)
{
  int currentAngle = measure_angle();
  if (currentAngle > 180)
  {
    currentAngle -= 360;
  }
  
  pControlSpeedR = initialSpeed + (currentAngle - targetAngle)*kp;
  if (pControlSpeedR > maxSpeed)
  {
    pControlSpeedR = maxSpeed;
  }
  if (pControlSpeedR < minSpeed)
  {
    pControlSpeedR = minSpeed;
  }
  
  pControlSpeedL = initialSpeed - (currentAngle - targetAngle)*kp;
  if (pControlSpeedL > maxSpeed)
  {
    pControlSpeedL = maxSpeed;
  }
  if (pControlSpeedL < minSpeed)
  {
    pControlSpeedL = minSpeed;
  }
  Serial.println(pControlSpeedL);
  Serial.println(pControlSpeedR);
}

void DriveForward (int initialSpeed, float targetAngle)
{
  SpeedControl(initialSpeed, targetAngle);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, pControlSpeedR);
  analogWrite(enB, pControlSpeedL);
}

void DriveBackward (int initialSpeed, float targetAngle)
{
  SpeedControl(initialSpeed, targetAngle);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, pControlSpeedR);
  analogWrite(enB, pControlSpeedL);
}

void TurnOff ()
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
