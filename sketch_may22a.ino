// Motor driver pin definitions
const int ENA = 9; // PWM pin for left motor speed
const int IN1 = 2; // Left motor forward
const int IN2 = 3; // Left motor backward
const int ENB = 10; // PWM pin for right motor speed
const int IN3 = 5; // Right motor forward
const int IN4 = 4; // Right motor backward

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set initial motor speeds to zero
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Execute command
    switch (command) {
      case 'F': // Move forward
        moveForward();
        break;
      case 'B': // Move backward
        moveBackward();
        break;
      case 'L': // Turn left
        turnLeft();
        delay(500);
        moveForward();
        break;
      case 'R': // Turn right
        turnRight();
        delay(500);
        moveForward();
        break;
      case 'S': // Stop
        stopMotors();
        break;
    }
  }
}

void moveForward() {
  analogWrite(ENA, 100); // Set speed to maximum (0-255)
  analogWrite(ENB, 100); // Set speed to maximum (0-255)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, 100); // Set speed to maximum (0-255)
  analogWrite(ENB, 100); // Set speed to maximum (0-255)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, 100); // Set speed to maximum (0-255)
  analogWrite(ENB, 100); // Set speed to maximum (0-255)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, 100); // Set speed to maximum (0-255)
  analogWrite(ENB, 100); // Set speed to maximum (0-255)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0); // Set speed to zero
  analogWrite(ENB, 0); // Set speed to zero
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
