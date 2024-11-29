// Servo Portion
#include <Servo.h>
const int servoPin = 12;
Servo myServo;

// Ultrasonic Sensor Portion
const int trigPin = 9;
const int echoPin = 10;

float duration;
float distanceCM;

void calculateDistance(){
  // Reset ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send out signal to measure time for return
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance (343 m/s = .034 cm/microsecond)
  distanceCM = (duration * 0.034) / 2;
}

void setup() {
  Serial.begin(9600);

  // Servo
  myServo.attach(servoPin);

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Rotate servo from 1 - 90 degrees
  for (int deg = 1; deg < 181; deg++){
    myServo.write(deg);
    delay(30);
    calculateDistance();

    Serial.print(deg);
    Serial.print(":");
    Serial.print(distanceCM);
    Serial.print("|");
  }

  // Rotate servo back from 90 - 1 degree
  for (int deg = 181; deg > 0; deg--){
    myServo.write(deg);
    delay(30);
    calculateDistance();

    Serial.print(deg);
    Serial.print(":");
    Serial.print(distanceCM);
    Serial.print("|");
  }
}
