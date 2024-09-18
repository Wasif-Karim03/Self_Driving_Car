#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();
//Add Buzzer on digital pin 6
const int buzzerPin = 6;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  
  if (!vl.begin()) {  // Check if the sensor initializes correctly
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("VL6180X sensor found");

  pinMode(buzzerPin, OUTPUT);  // Set buzzer pin as output
  digitalWrite(buzzerPin, LOW);  // Ensure the buzzer is off initially
}

void loop() {
  uint8_t distance = vl.readRange();  // Measure the distance
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" mm");

  if (distance <= 250) {  // Check if the distance is within the threshold
    int beepDelay = map(distance, 0, 180, 10, 500);  // Calculate beep delay based on distance
    
    digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
    delay(70);  // Duration of the beep
    digitalWrite(buzzerPin, LOW);  // Turn off the buzzer
    delay(beepDelay);  // Delay before the next beep based on distance
  } else {
    digitalWrite(buzzerPin, LOW);  // Ensure the buzzer is off if distance is greater than threshold
  }

  delay(50);  // Short delay before the next reading
}
