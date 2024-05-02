#include <Servo.h>

Servo pirServo; // Servo for PIR motion detection (not used in this code)
Servo ultraServo; // Servo for ultrasonic sensor

#define ultraServoPin 6 // Pin for servo ultrasonik

#define trigPin 10
#define echoPin 13
#define pirPin 7

int pos = 0;
float duration, distance;
float averageDistance = 0;
const int numAverageReadings = 3; // Number of readings to average
const long interval = 3000;        // Interval in milliseconds
const int numServoMovements = 3;   // Number of times servo moves

unsigned long previousMillis = 0;
float previousDistance = 0; // Initialize previousDistance

bool motionState = false; // We start with no motion detected.

void setup() {
  Serial.begin(9600);
  ultraServo.attach(ultraServoPin);
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
}


void moveUltraServo() {
  for (int i = 0; i < numServoMovements; i++) {
    ultraServo.write(0);
    delay(1000);
    ultraServo.write(180);
    delay(750);
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
      previousMillis = currentMillis;  // Reset timer on motion detection
      averageDistance = 0;            // Reset average distance
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED.
    if (motionState == true) {
      Serial.println("Motion ended!");
      motionState = false;
    }
  }

  if (motionState == true && currentMillis - previousMillis >= interval) {
    float totalDistance = 0;
    for (int i = 0; i < numAverageReadings; i++) {
      ultraSound();
      totalDistance += distance;
      delay(10);
    }
    averageDistance = totalDistance / numAverageReadings;

    if (abs(averageDistance - previousDistance) >= 5) {
      Serial.println("Significant distance change detected!");
      moveUltraServo(); // Call function to move ultra servo
    }

    previousDistance = averageDistance;
  }
}
