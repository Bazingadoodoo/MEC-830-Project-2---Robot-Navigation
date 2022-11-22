//IMU--------------------------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float angle_offset;
float angle;
float angle_offset_mapped;

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

void ISR_Right() {
  if (Right) {
    rightCounter++;
  }
  else {
    rightCounter--;
  }
}

void ISR_Left() {
  if (Left) {
    leftCounter++;
  }
  else {
    leftCounter--;
  }
}

void RotateRight(int motorSpeedL, int motorSpeedR, float startAngle, float rotation_angle)
{
  float Z = measure_angle();
  float Z_mapped = Z + 180;
  if (Z_mapped >= 360)
  {
    Z_mapped = Z_mapped - 360;
  }
  while ((Z_mapped - startAngle) < (rotation_angle - 32))
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, motorSpeedR);
    analogWrite(enB, motorSpeedL);
    //-----------------------------------------------------------------

    Z = measure_angle();      // when updating Z_mapped this way,
    Z_mapped = Z + 180;       // the while loop cannot properly
    if (Z_mapped >= 360)            // determine when to execute and
    { // when to stop
      Z_mapped = Z_mapped - 360;
    }

    //-----------------------------------------------------------------
    //Z_mapped++;                   // when updating Z_mapped this way,
    // the while loop functions normally
    //-----------------------------------------------------------------
    Serial.print("Z_mapped: ");
    Serial.print(Z_mapped);
    Serial.print("        //  ");
    Serial.print("startAngle: ");
    Serial.println(startAngle);
    Serial.print("rotation_angle: ");
    Serial.print(rotation_angle);
    Serial.print("  //  ");
    Serial.print("Angle Difference: ");
    Serial.println(Z_mapped - startAngle);
    delay(100);
  }
  if ((Z_mapped - startAngle) >= rotation_angle - 32)
  {
    TurnOff();
    Serial.println("off");
    float X = measure_angle() + 180;
    Serial.println(X);
  }
}

void TurnRight(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 1;
  Left = 0;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
}

void TurnLeft(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 0;
  Left = 1;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
}

void DriveForward(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 1;
  Left = 1;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
}

void DriveBackward(int motorSpeedL, int motorSpeedR, double delayTime) {
  Right = 0;
  Left = 0;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeedR);
  analogWrite(enB, motorSpeedL);
  delay(delayTime);
  TurnOff();
}

void TurnOff() {
  Right = 0;
  Left = 0;
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
  float SR = ((rightCounter - offsetR) / 20) * (2 * PI * R);
  float SL = ((leftCounter - offsetL) / 20) * (2 * PI * R);
  float meanDistance = (SR + SL) / 2;
  return meanDistance;
}

void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), ISR_Left, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), ISR_Right, RISING);
  irReceive.enableIRIn();
  Serial.println();
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
  angle_offset = event.orientation.x; //get z-axis rotation angle
  angle_offset_mapped = angle_offset + 180;
  if (angle_offset_mapped > 360)
  {
    angle_offset_mapped = angle_offset_mapped - 360;
  }
  Serial.print("Offset: ");
  Serial.println(angle_offset_mapped);
}

void loop()
{
  if (irReceive.decode(&irInput))
  {
    int irReading = irInput.value;
    switch (irReading)
    {
      case 6375:                // button 2
        DriveForward(123, 100, 1000);
        break;
      case 19125:               // button 8
        DriveBackward(113, 95, 1000);
        break;
      case 4335:                // button 4
        break;
      case 23205:               // button 6
        RotateRight(100, 71, angle_offset_mapped, 90);
        break;
      case 14535:               // button 5
        TurnOff();
        break;
      case 12495:               // button 1
        TurnLeft(0, 100, 300);
        break;
      case 31365:               // button 3
        TurnRight(120, 0, 300);
        break;
    }
    irReceive.resume();
    //Serial.println(irReading);
  }
  /*
    Serial.print("left counter: ");
    Serial.print(leftCounter);
    Serial.print("  //  ");
    Serial.print("right counter: ");
    Serial.println(rightCounter);
  */
}
