#include <Servo.h>

Servo myservo;

#define servoPin 9
#define trigPin 10
#define echoPin 13

int pos = 0;
float duration, distance;
int numReadings = 0;
float totalDistance = 0;
float averageDistance = 0;
const int numAverageReadings = 5; // Number of readings to average
const long interval = 5000;        // Interval in milliseconds
const int numServoMovements = 3;   // Number of times servo moves

unsigned long previousMillis = 0;
float previousDistance = 0;

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void ultraSound() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0343;

  // Accumulate total distance and increment number of readings
  totalDistance += distance;
  numReadings++;

  if (distance >= 400 || distance <= 2) {
    Serial.println("Out of range");
  } else {
    Serial.print("Detected Something at ");
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");
  }
}

void moveServo() {
  // Move the servo several times
  for (int i = 0; i < numServoMovements; i++) {
    myservo.write(90);  // Move servo to 90 degrees
    delay(500);         // Delay for 0.5 seconds
    myservo.write(0);   // Move servo back to 0 degrees
    delay(500);         // Delay for 0.5 seconds
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Reset total distance and number of readings for averaging
    totalDistance = 0;
    numReadings = 0;

    // Take multiple readings for averaging
    for (int i = 0; i < numAverageReadings; i++) {
      ultraSound();
      delay(10); // Small delay between readings
    }

    // Calculate average distance
    if (numReadings > 0) {
      averageDistance = totalDistance / numReadings;
    }

    // Check if average distance changed significantly
    if (abs(averageDistance - previousDistance) >= 5) {
      Serial.println("Movement detected!");
      moveServo(); // Call function to move servo
    }

    // Update previous distance
    previousDistance = averageDistance;
  }
}
