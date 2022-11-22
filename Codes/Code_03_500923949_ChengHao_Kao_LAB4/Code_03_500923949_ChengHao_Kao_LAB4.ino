#include<Servo.h>

Servo myservo;

#define echoPin 5
#define trigPin 4

float duration;
float distance_cm;
int pos = 0;

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(1);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(6);
  Serial.println("Radar Start");
  //myservo.write(180);
}

void loop()
{
  for (int i = 0; i < 180; i++)
  {
    pos += 1;
    myservo.write(pos);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance_cm = duration * 0.034 / 2.0;
    Serial.print(pos);
    Serial.print(",");
    Serial.println(distance_cm);
    delay(50);
  }
  for (int i = 180; i > 0; i--)
  {
    pos -= 1;
    myservo.write(pos);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance_cm = duration * 0.034 / 2;
    Serial.print(pos);
    Serial.print(",");
    Serial.println(distance_cm);
    delay(50);
  }
}
