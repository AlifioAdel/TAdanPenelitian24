#include <Servo.h>

Servo myservo;

#define servoPin 9
#define pirPin 7

int pos = 0;
const long interval = 5000;        // Interval in milliseconds
const int numServoMovements = 3;   // Number of times servo moves

unsigned long previousMillis = 0;

bool motionState = false; // We start with no motion detected.

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(pirPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
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

}
