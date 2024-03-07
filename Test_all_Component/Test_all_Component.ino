// Set the included library
#include <Servo.h> 

Servo myservo; // Create Object servo

#define servoPin 9  // Define the pin for servo. Used Pin 9 PWM
#define trigUPin 10 // Define the pin for servo. Used Pin 10 PWM
#define echoUPin 13 // Define the pin for servo. Used Pin 13

int pos = 0; // Create variable for storing the servo position
float duration, distance; // Create variable for storing the duration and distance value

void setup() {
  Serial.begin(9600); // Open the Serial Port
  myservo.attach(servoPin); // Attach servo to the set pin
  // Set pin mode for the ultrasonic sensor
  pinMode(trigUPin, OUTPUT);
  pinMode(echoUPin, INPUT);
}

void loop() {
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
  
  // Send results to Serial Monitor
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
