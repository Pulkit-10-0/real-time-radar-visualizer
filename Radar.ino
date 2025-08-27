#include <Servo.h>

Servo myServo;
const int trigPin = 9;
const int echoPin = 10;

int angle = 0;
int step = 1;

void setup() {
  Serial.begin(9600);
  myServo.attach(6); // Servo signal pin
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // cm
}

void loop() {
  myServo.write(angle);
  delay(30);

  long distance = getDistance();
  if (distance > 200) distance = 200; // clamp max range

  Serial.print(angle);
  Serial.print(",");
  Serial.println(distance);

  angle += step;
  if (angle >= 180 || angle <= 0) step = -step;
}
