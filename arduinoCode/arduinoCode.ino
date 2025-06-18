// Initiate Servos
#include <Servo.h>

Servo servoRight;
Servo servoLeft;
Servo servoFront;
Servo servoBack;

// ========== PIN CONFIGURATION ==========
// Front IR Sensors (Left to Right)
const int frontSensors[8] = {38, 40, 42, 44, 46, 48, 50, 52};

// Back IR Sensors (Left to Right) 
const int backSensors[8] = {51, 49, 47, 45, 43, 41, 39, 37};

// Right side IR sensor for table counting
const int rightTableSensor = 24;
const int leftTableSensor = 26;

// L298N Motor Driver Pins
const int motorA_Pin1 = 23;  // Left Motor
const int motorA_Pin2 = 25;
const int motorA_Enable = 2;

const int motorB_Pin1 = 27;  // Right Motor
const int motorB_Pin2 = 29;
const int motorB_Enable = 3;

// Front Ultrasonic
const int trigFront = 7;
const int echoFront = 6;

// Back Ultrasonic
const int trigBack = 5;
const int echoBack = 4;

// Servo pin
const int servoFrontPin = 11;
const int servoBackPin = 8;
const int servoLeftPin = 10;
const int servoRightPin = 9;

int sweepMin = 60;
int sweepMax = 120;
int currentAngleFront = sweepMin;
int currentAngleBack = sweepMin;
bool sweepingForwardFront = true;
bool sweepingForwardBack = true;
const int sweepDelay = 30;
unsigned long lastSweepTime = 0;

//seven segment
int segmentPins[8] = {9, 10, 14, 12, 11, 7, 6, 15}; // a b c d e f g dp

// Format: a b c d e f g dp
byte digitCodes[10][8] = {
  {1, 1, 1, 1, 1, 1, 0, 0}, // 0
  {0, 1, 1, 0, 0, 0, 0, 0}, // 1
  {1, 1, 0, 1, 1, 0, 1, 0}, // 2
  {1, 1, 1, 1, 0, 0, 1, 0}, // 3
  {0, 1, 1, 0, 0, 1, 1, 0}, // 4
  {1, 0, 1, 1, 0, 1, 1, 0}, // 5
  {1, 0, 1, 1, 1, 1, 1, 0}, // 6
  {1, 1, 1, 0, 0, 0, 0, 0}, // 7
  {1, 1, 1, 1, 1, 1, 1, 0}, // 8
  {1, 1, 1, 1, 0, 1, 1, 0}  // 9
};

// ========== ROBOT CONFIGURATION ==========
const int baseSpeed = 150;        // Base motor speed (0-255)
const int maxSpeed = 200;         // Maximum motor speed
const int minSpeed = 100;         // Minimum motor speed

// PID Constants (tune these for your robot)
const float Kp = 30.0;    // Proportional gain
const float Ki = 0.0;     // Integral gain  
const float Kd = 20.0;    // Derivative gain

// ========== GLOBAL VARIABLES ==========
int targetTable = 0;
int currentTable = 0;
bool isDelivering = true;  // true = going to table, false = returning
bool lastRightSensorState = false;
bool lastLeftSensorsState = false;
bool currentRightSensorState = false;
bool currentLeftSensorState = false;

// PID variables
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// Sensor weights for line position calculation
const int sensorWeights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};


const int servoLeftDefault = 0;
const int servoRightDefault = 90;

long safeDistance = 20; //Safe Distance (cm)
//
void setup() {

  Serial.begin(115200);

  // Initialize sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(frontSensors[i], INPUT);
    pinMode(backSensors[i], INPUT);
  }
  pinMode(rightTableSensor, INPUT);
  pinMode(leftTableSensor, INPUT);
  
  // Initialize motor pins
  pinMode(motorA_Pin1, OUTPUT);
  pinMode(motorA_Pin2, OUTPUT);
  pinMode(motorA_Enable, OUTPUT);
  pinMode(motorB_Pin1, OUTPUT);
  pinMode(motorB_Pin2, OUTPUT);
  pinMode(motorB_Enable, OUTPUT);

  // Initailize ultrasonic pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigBack, OUTPUT);
  pinMode(echoBack, INPUT);

  // Initialize servo pins
  servoRight.attach(servoLeftPin);
  servoLeft.attach(servoRightPin);
  servoFront.attach(servoFrontPin);
  servoBack.attach(servoBackPin);


  servoRight.write(servoLeftDefault);
  servoLeft.write(servoRightDefault);
  servoFront.write(currentAngleFront);
  servoBack.write(currentAngleBack);
  
  // Stop motors initially
  stopMotors();

  // Initialize seven segment pins
   for (int i = 0; i < 8; i++) 
   {
    pinMode(segmentPins[i], OUTPUT);
   }
}


// ========== MAIN LOOP ==========
void loop() {
  // Check for ESP communication
  handleESPCommunication();
  
  // If we have a target table, start line following
  if (targetTable > 0) {
    if (isObstacleDetected()) {
      stopMotors(); // motor berhenti
      return;
    }

    sweepServo();       // servo gerak hanya jika tidak ada halangan
    followLine();
    countTables();
  }

  
  delay(10); // Small delay for stability

}


// ========== ESP COMMUNICATION ==========
void handleESPCommunication() {
  if (Serial.available()) {
    String feedback = Serial.readStringUntil('\n');
    feedback.trim();
    
    if (feedback.startsWith("targetTable:")) {
      targetTable = feedback.substring(feedback.indexOf(":") + 1).toInt();
      currentTable = 0;
      isDelivering = true;
      
      Serial.println("mode:delivering");
    }
  }
}

// ========== LINE FOLLOWING WITH PID ==========
void followLine() {
  int* activeSensors = isDelivering ? (int*)frontSensors : (int*)backSensors;
  // Read sensor values
  bool sensorValues[8];
  int activeSensorCount = 0;
  
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(activeSensors[i]) == LOW; // LOW = black line detected
    if (sensorValues[i]) activeSensorCount++;
  }
  
  // Check if robot is off the line
  if (activeSensorCount == 0) {
    // No line detected - stop or continue with last known direction
    stopMotors();
    return;
  }
  
  // Calculate line position (-7 to +7, 0 = center)
  float position = calculateLinePosition(sensorValues);
  
  // PID calculation
  float error = position;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  if (deltaTime > 0) {
    integral += error * deltaTime;
    float derivative = (error - lastError) / deltaTime;
    
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Apply correction to motor speeds
    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;
    
    // Constrain speeds
    leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
    
    // Move motors
    if (isDelivering) {
      // Serial.println("majuu");
      moveForward(leftSpeed, rightSpeed);
    } else {
      // Serial.println("mundurr");
      moveBackward(leftSpeed, rightSpeed);
    }
    
    lastError = error;
    lastTime = currentTime;
  }
}

// ========== LINE POSITION CALCULATION ==========
float calculateLinePosition(bool sensorValues[8]) {
  float weightedSum = 0;
  int activeCount = 0;
  
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i]) {
      weightedSum += sensorWeights[i];
      activeCount++;
    }
  }
  
  if (activeCount > 0) {
    return weightedSum / activeCount;
  }
  return 0; // Default to center if no sensors active
}

// ========== TABLE COUNTING ==========
void countTables() {
  currentRightSensorState = digitalRead(rightTableSensor) == LOW; // LOW = black detected
  currentLeftSensorState = digitalRead(leftTableSensor) == LOW; // LOW = black detected
  
  // Detect rising edge (transition from white to black)
  if (currentRightSensorState && !lastRightSensorState && currentLeftSensorState && !lastLeftSensorsState) {
    if (isDelivering) {
      currentTable++;
      Serial.print("currentTable:");
      Serial.println(currentTable);
      
      // Check if we've reached the target table
      if (currentTable >= targetTable) {
        stopMotors();
        Serial.println("mode:serving");
        delay(1000); // Pause before returning
 
        servingFood();

        isDelivering = false;
        delay(1000); // Pause before returning
        Serial.println("mode:returning");
      }
    } else {
      // Returning mode - decrement table count
      currentTable--;
      Serial.print("currentTable:");
      Serial.println(currentTable);
      
      // Check if we've returned to start
      if (currentTable <= 0) {
        stopMotors();
        Serial.println("mode:idle");
        targetTable = 0; // Reset for next delivery
        Serial.println("Delivery complete! Ready for next order.");
      }
    }
  }
  
  lastRightSensorState = currentRightSensorState;
  lastLeftSensorsState = currentLeftSensorState;
}

// ========== MOTOR CONTROL FUNCTIONS ==========
void moveForward(int leftSpeed, int rightSpeed) {
  // Left motor forward
  digitalWrite(motorA_Pin1, HIGH);
  digitalWrite(motorA_Pin2, LOW);
  analogWrite(motorA_Enable, leftSpeed);
  
  // Right motor forward
  digitalWrite(motorB_Pin1, HIGH);
  digitalWrite(motorB_Pin2, LOW);
  analogWrite(motorB_Enable, rightSpeed);
}

void moveBackward(int leftSpeed, int rightSpeed) {
  // Left motor backward
  digitalWrite(motorA_Pin1, LOW);
  digitalWrite(motorA_Pin2, HIGH);
  analogWrite(motorA_Enable, leftSpeed);
  
  // Right motor backward
  digitalWrite(motorB_Pin1, LOW);
  digitalWrite(motorB_Pin2, HIGH);
  analogWrite(motorB_Enable, rightSpeed);
}

void stopMotors() {
  digitalWrite(motorA_Pin1, LOW);
  digitalWrite(motorA_Pin2, LOW);
  analogWrite(motorA_Enable, 0);

  digitalWrite(motorB_Pin1, LOW);
  digitalWrite(motorB_Pin2, LOW);
  analogWrite(motorB_Enable, 0);

}

// ========== UTILITY FUNCTIONS ==========
void printSensorValues() {
  // Debug function to print sensor readings
  Serial.print("Front: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(digitalRead(frontSensors[i]));
    Serial.print(" ");
  }
  Serial.print(" | Back: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(digitalRead(backSensors[i]));
    Serial.print(" ");
  }
  Serial.print(" | Right: ");
  Serial.println(digitalRead(rightTableSensor));
}

// ========== SERVOS FUNCTIONS ==========
void servingFood(){
  int servoSteps = 2; 
  int delayBetweenSteps = 30;
  
  // Move servos asynchronously
  for (int i = 0; i <= 90; i += servoSteps) {
    servoRight.write(i); 
    servoLeft.write(90 - i);      
    delay(delayBetweenSteps);
  }

  delay(500);

  // Return to starting position
  for (int i = 0; i <= 90; i += servoSteps) {
    servoRight.write(90 - i);      
    servoLeft.write(i); 
    delay(delayBetweenSteps);
  }
}

void sweepServo() {
  if (millis() - lastSweepTime >= sweepDelay) {
    lastSweepTime = millis();

    if (isDelivering) {
      if (sweepingForwardFront) {
        currentAngleFront += 2;
        if (currentAngleFront >= sweepMax) sweepingForwardFront = false;
      } else {
        currentAngleFront -= 2;
        if (currentAngleFront <= sweepMin) sweepingForwardFront = true;
      }
      servoFront.write(currentAngleFront);
    } else {
      if (sweepingForwardBack) {
        currentAngleBack += 2;
        if (currentAngleBack >= sweepMax) sweepingForwardBack = false;
      } else {
        currentAngleBack -= 2;
        if (currentAngleBack <= sweepMin) sweepingForwardBack = true;
      }
      servoBack.write(currentAngleBack);
    }
  }
}


long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // max timeout 30ms
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

bool isObstacleDetected() {
  long distance;
  if (isDelivering) {
    distance = readUltrasonic(trigFront, echoFront);
  } else {
    distance = readUltrasonic(trigBack, echoBack);
  }

  if (distance > 0 && distance < safeDistance) {
    Serial.print("Obstacle detected at ");
    Serial.print(distance);
    Serial.println(" cm");
    return true;
  }
  return false;
}

// ========== CONFIGURATION FUNCTIONS ==========
void setPIDValues(float p, float i, float d) {
  // Function to update PID values during runtime
  // Uncomment and modify const declarations to variables if needed
  /*
  Kp = p;
  Ki = i;
  Kd = d;
  Serial.println("PID values updated!");
  */
}

void setMotorSpeeds(int base, int max, int min) {
  // Function to update motor speeds during runtime
  // Uncomment and modify const declarations to variables if needed
  /*
  baseSpeed = base;
  maxSpeed = max;
  minSpeed = min;
  Serial.println("Motor speeds updated!");
  */
}