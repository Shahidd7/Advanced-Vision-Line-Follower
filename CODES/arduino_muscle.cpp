/*
 * Advanced Line Follower - Arduino Muscle
 * Listens for serial commands formatted as "LeftSpeed,RightSpeed\n"
 * Baud Rate: 115200
 */

#define PWM1  9
#define DIR1  8
#define PWM2  10
#define DIR2  7

void setMotor(int pwmPin, int dirPin, int speed) {
  // Negative speed flips the direction pin for reverse/spinning
  if (speed < 0) {
    digitalWrite(dirPin, LOW); // Swap to HIGH if wheels spin the wrong way
    analogWrite(pwmPin, constrain(abs(speed), 0, 255));
  } else {
    digitalWrite(dirPin, HIGH); // Swap to LOW if wheels spin the wrong way
    analogWrite(pwmPin, constrain(speed, 0, 255));
  }
}

void setup() {
  Serial.begin(115200); 
  pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
  
  // Start safely stopped
  setMotor(PWM1, DIR1, 0);
  setMotor(PWM2, DIR2, 0);
}

void loop() {
  // Await specific string format: "Left,Right\n"
  if (Serial.available() > 0) {
    int leftSpeed = Serial.parseInt();
    int rightSpeed = Serial.parseInt();
    
    if (Serial.read() == '\n') {
      setMotor(PWM1, DIR1, leftSpeed);
      setMotor(PWM2, DIR2, rightSpeed);
    }
  }
}
