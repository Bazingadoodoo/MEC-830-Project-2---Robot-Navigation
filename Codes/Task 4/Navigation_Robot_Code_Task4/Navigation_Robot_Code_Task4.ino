//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;

// motor driver----------------------------
#define enA 9       // right wheel
#define enB 6       // left wheel
#define in1 12      // forward
#define in2 13      // backward
#define in3 7       // backward
#define in4 8       // forward

void TurnOff() {                                                        // Turn Off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

float measure_angle(void)                                               // Angle Measurement
{
  float z_angle;// to determine absolute orientation
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  z_angle = event.orientation.x; //get z-axis rotation angle
  Serial.println(z_angle);
  return z_angle;
}

void forward ()                                                         // Forward
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backward ()                                                        // Backward
{
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void left ()                                                            // Turn Left
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
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

void DriveForward ()                                                    // Drive Forward
{
  forward();
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

void SpeedControlRotation ()                                            // Speed Control Rotation
{
  currentAngle = measure_angle();
  if ((targetAngle == 0) | (targetAngle == 90))
  {
    if ((currentAngle > 270) && (currentAngle < 360))
    {
      currentAngle -= 360;
    }
    Serial.print("current: ");
    Serial.println(currentAngle);
  }
  float error = currentAngle - targetAngle;
  if (error < -1)
  {
    pControlSpeed = initialSpeed - (currentAngle - targetAngle) * kp2;
    //pControlSpeedR = initialSpeed - (currentAngle - targetAngle) * kp2;
    //pControlSpeedL = initialSpeed - (currentAngle - targetAngle) * kp2;
    CW = 1; CCW = 0; stopRotate = 0;
    Serial.println(CW);
  }
  else if (error > 1)
  {
    pControlSpeed = initialSpeed + (currentAngle - targetAngle) * kp2;
    //pControlSpeedR = initialSpeed + (currentAngle - targetAngle) * kp2;
    //pControlSpeedL = initialSpeed + (currentAngle - targetAngle) * kp2;
    CW = 0; CCW = 1; stopRotate = 0;
    Serial.println(CCW);
  }
  else
  {
    pControlSpeed = 0;
    CW = 0; CCW = 0; stopRotate = 1;
    Serial.println(stopRotate);
  }
  if (pControlSpeed > maxSpeed2)
  {
    pControlSpeed = maxSpeed2;
  }
  if (pControlSpeed < minSpeed2)
  {
    pControlSpeed = minSpeed2;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Calibrating IMU");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the imu */

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
}
}

void loop() {
}
