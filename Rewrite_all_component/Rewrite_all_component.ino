#include <Servo.h>

Servo pirServo; // Servo for PIR motion detection (not used in this code)
Servo ultraServo; // Servo for ultrasonic sensor

#define ultraServoPin 6 // Pin for servo ultrasonik
#define trigPin 10
#define echoPin 13
#define pirPin 7

int pos = 0;
float duration, distance;

const long interval = 1000;      //Interval in milliseconds
const int numServoMovements = 4; //Number of times servo moves

unsigned long previousMillis = 0;
float newDistance, previousDistance = 0; // Initialize previousDistance

bool motionDetected = false; // We start with no motion detected.

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
  newDistance = (duration / 2) * 0.0343;
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

  if (val == HIGH && !motionDetected) {
    digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED.
    Serial.println("Motion detected!");
    motionDetected = true;
    previousMillis = currentMillis;  // Reset timer on motion detection
    //averageDistance = 0;            // Reset average distance
    moveUltraServo();
  } else if (val == LOW && motionDetected) {
    digitalWrite(LED_BUILTIN, LOW); // Turn off the LED.
    Serial.println("Motion ended!");
    motionDetected = false;
  }

  if (motionDetected) {
    ultraSound();
    Serial.print("raw distance: ");
    Serial.println(newDistance);

    if (abs(newDistance - previousDistance) >= 5) {
      Serial.println("Significant distance change detected!");
      // Serial.print("New distance: ");
      // Serial.println(newDistance);
      moveUltraServo(); // Call function to move ultra servo
    }

    previousDistance = newDistance;
  }
}
