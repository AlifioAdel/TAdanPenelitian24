#include <Servo.h>

Servo myservo;

#define servoPin 9
#define trigPin 10
#define echoPin 13
#define pirPin 7

int pos = 0;
float duration, distance;
int numReadings = 0;
float totalDistance = 0;
float averageDistance = 0;
const int numAverageReadings = 3; // Number of readings to average
const long interval = 3000;        // Interval in milliseconds
const int numServoMovements = 3;   // Number of times servo moves

unsigned long previousMillis = 0;
float previousDistance = 0; // Initialize previousDistance

bool motionState = false; // We start with no motion detected.

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pirPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void ultraSound() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0343;

  totalDistance += distance;
  numReadings++;

  Serial.print("Detected Something at ");
  Serial.print("Distance = ");
  if (distance >= 400 || distance <= 2) {
    Serial.println("Out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }
}

void moveServo() {
  for (int i = 0; i < numServoMovements; i++) {
    myservo.write(90);
    delay(500);
    myservo.write(0);
    delay(500);
  }
}

void loop() {
  unsigned long currentMillis = millis();
  int val = digitalRead(pirPin);

  if (val == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED.
    if (motionState == false) {
      Serial.println("Motion detected!");
      motionState = true;
      moveServo(); // Call function to move servo once
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED.
    if (motionState == true) {
      Serial.println("Motion ended!");
      motionState = false;
    }
  }

  if (motionState == true && currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    totalDistance = 0;
    numReadings = 0;

    for (int i = 0; i < numAverageReadings; i++) {
      ultraSound();
      delay(10);
    }

    if (numReadings > 0) {
      averageDistance = totalDistance / numReadings;
    }

    if (abs(averageDistance - previousDistance) >= 5) {
      Serial.println("Movement detected!");
      moveServo(); // Call function to move servo
    }

    previousDistance = averageDistance;
  }
}
