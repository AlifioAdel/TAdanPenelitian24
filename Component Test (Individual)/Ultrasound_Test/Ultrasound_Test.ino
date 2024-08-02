/*
  Ultrasound sensor test
  DONE
*/

// This uses Serial Monitor to display Range Finder distance readings
// #include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin 10, Echo to Arduino pin 13

#define trigPin 10
#define echoPin 13
#define max_distance 400

float duration, distance;
// float distance;
// NewPing sonar(trigPin, echoPin, max_distance);

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void Ultrasound(){
  // Write a pulse to the HC-SR04 Trigger Pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the response from the HC-SR04 Echo Pin
  duration = pulseIn(echoPin, HIGH);
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  distance = (duration / 2) * 0.0343;

}

void loop() {
  Ultrasound();
  // Send results to Serial Monitor
  // distance = sonar.ping_cm();
  Serial.print("Detected Something at ");
  Serial.print("Distance = ");
  if (distance >= 400 || distance <= 2) {
     Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");
    delay(500);
  }
  delay(500);
}