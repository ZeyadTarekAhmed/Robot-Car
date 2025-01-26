#define enA 10  // Enable1 L298 Pin enA
#define in1 9   // Motor1 L298 Pin in1
#define in2 8   // Motor1 L298 Pin in2
#define in3 7   // Motor2 L298 Pin in1
#define in4 6   // Motor2 L298 Pin in1
#define enB 5   // Enable2 L298 Pin enB
#define servo 13 // Digital pin for servo
#define R_S 11  // Digital pin for IR sensor Right
#define L_S 12  // Digital pin for IR sensor Left
#define echo 2  // Echo pin for ultrasonic sensor
#define trigger 3 // Trigger pin for ultrasonic sensor

int distance_L, distance_F, distance_R;
long distance;
int set = 20; // Threshold distance for obstacle avoidance
int bt_ir_data; // Variable to receive data from the serial port
int Speed = 130;
int mode = 0;

void setup() { 
  pinMode(R_S, INPUT); // IR sensor Right
  pinMode(L_S, INPUT); // IR sensor Left
  pinMode(echo, INPUT); // Ultrasonic Echo
  pinMode(trigger, OUTPUT); // Ultrasonic Trigger
  pinMode(enA, OUTPUT); // L298 Pin enA
  pinMode(in1, OUTPUT); // L298 Pin in1
  pinMode(in2, OUTPUT); // L298 Pin in2
  pinMode(in3, OUTPUT); // L298 Pin in3
  pinMode(in4, OUTPUT); // L298 Pin in4
  pinMode(enB, OUTPUT); // L298 Pin enB

  Serial.begin(9600); // Serial communication at 9600bps

  pinMode(servo, OUTPUT);
  servoPulse(servo, 70); // Initialize servo to center
}

void loop() {
  if (Serial.available() > 0) { // Read data from serial port
    bt_ir_data = Serial.read();
    Serial.println(bt_ir_data);
    if (bt_ir_data > 20) {
      Speed = bt_ir_data;
    }
  }

  if (bt_ir_data == 8) {
    mode = 0;
    Stop();
  } else if (bt_ir_data == 9) {
    mode = 1;
    Speed = 130;
  } else if (bt_ir_data == 10) {
    mode = 2;
    Speed = 255;
  }

  analogWrite(enA, Speed); // Set motor speeds
  analogWrite(enB, Speed);

  if (mode == 0) {
    // Manual control
    manualControl(bt_ir_data);
  }

  if (mode == 1) {
    // Line follower mode
    lineFollower();
  }

  if (mode == 2) { // Obstacle Avoidance Mode
    distance_F = Ultrasonic_read();
    Serial.print("S="); // Output front distance for debugging
    Serial.println(distance_F);
    if (distance_F > 20) {
      forword(); // Move forward if no obstacle in front
    } else {
      Check_side(); // Check sides if an obstacle is detected in front
    }
  }
  delay(10);
}

// Manual control commands
void manualControl(int command) {
  switch (command) {
    case 1: forword(); break;
    case 2: backword(); break;
    case 3: turnLeft(); break;
    case 4: turnRight(); break;
    case 5: Stop(); break;
    case 6: turnLeft(); delay(400); Stop(); break;
    case 7: turnRight(); delay(400); Stop(); break;
    default: Stop(); break;
  }
}

// Line follower logic
void lineFollower() {
  if ((digitalRead(R_S) == LOW) && (digitalRead(L_S) == LOW)) {
    forword();
  } else if ((digitalRead(R_S) == HIGH) && (digitalRead(L_S) == LOW)) {
    turnRight();
  } else if ((digitalRead(R_S) == LOW) && (digitalRead(L_S) == HIGH)) {
    turnLeft();
  } else {
    Stop();
  }
}

// Check side distances to avoid obstacles

void Check_side(){
    Stop();
    delay(100);
 for (int angle = 70; angle <= 140; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    distance_L = Ultrasonic_read();
    Serial.print("l=");
    Serial.println( distance_L);
    delay(100);
  for (int angle = 140; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle); 
    }
    delay(500);
    distance_R = Ultrasonic_read();
    Serial.print("r=");
    Serial.println( distance_R);
    delay(100);
 for (int angle = 0; angle <= 70; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    compareDistance();
}
void compareDistance(){
       if (distance_L > distance_R){
  turnLeft();
  delay(530);
  }
  else if (distance_R > distance_L){
  turnRight();
  delay(530);
  }
  else{
  backword();
  delay(300);
  turnRight();
  delay(600);
  }
}
// Ultrasonic read function
int Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  distance = pulseIn(echo, HIGH);
  return distance / 57; // Return distance in cm
}

// Servo control function
void servoPulse (int pin, int angle){
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50);                   // Refresh cycle of servo
}

// Motor control functions
void forword() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backword() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
