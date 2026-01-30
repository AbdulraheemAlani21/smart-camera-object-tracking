#include <Servo.h>

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);
  panServo.attach(9);
  tiltServo.attach(10);
}

void loop() {
  if (Serial.available()) {

    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) {

      int panAngle = data.substring(0, commaIndex).toInt();
      int tiltAngle = data.substring(commaIndex + 1).toInt();

      // Constrain angles to servo-safe range
      panAngle = constrain(panAngle, 0, 180);
      tiltAngle = constrain(tiltAngle, 0, 180);

      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
    }
  }
}
