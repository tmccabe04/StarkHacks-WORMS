
// Motor A
const int IN1 = 11;
const int IN2 = 12;

// Motor B
const int IN3 = 13;
const int IN4 = 14;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Move Forward
  // Motor A forward: IN1 HIGH, IN2 LOW
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Motor B forward: IN3 HIGH, IN4 LOW
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  delay(2000); // Run for 2 seconds
}
