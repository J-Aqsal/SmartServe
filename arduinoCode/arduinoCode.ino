#define PB1 22
#define PB2 24
#define PB3 26
#define PB4 28
#define LED_PIN 2
// ========== PIN CONFIGURATION ==========
// Front IR Sensors (Left to Right)
const int frontSensors[8] = {2, 3, 4, 5, 6, 7, 8, 9};

// Back IR Sensors (Left to Right) 
const int backSensors[8] = {10, 11, 12, 13, A0, A1, A2, A3};

// Right side IR sensor for table counting
const int rightTableSensor = A4;

// L298N Motor Driver Pins
const int motorA_Pin1 = 22;  // Left Motor
const int motorA_Pin2 = 23;
const int motorA_Enable = 24;

const int motorB_Pin1 = 25;  // Right Motor
const int motorB_Pin2 = 26;
const int motorB_Enable = 27;

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
bool currentRightSensorState = false;

// PID variables
float lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// Sensor weights for line position calculation
const int sensorWeights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

void setup() {

  Serial.begin(115200);

  // Initialize sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(frontSensors[i], INPUT);
    pinMode(backSensors[i], INPUT);
  }
  pinMode(rightTableSensor, INPUT);
  
  // Initialize motor pins
  pinMode(motorA_Pin1, OUTPUT);
  pinMode(motorA_Pin2, OUTPUT);
  pinMode(motorA_Enable, OUTPUT);
  pinMode(motorB_Pin1, OUTPUT);
  pinMode(motorB_Pin2, OUTPUT);
  pinMode(motorB_Enable, OUTPUT);
  
  // Stop motors initially
  stopMotors();
}


// ========== MAIN LOOP ==========
void loop() {
  // Check for ESP communication
  handleESPCommunication();
  
  // If we have a target table, start line following
  if (targetTable > 0) {
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
      
      Serial.print("Target table set to: ");
      Serial.println(targetTable);
      Serial.println("Starting delivery...");
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
      moveForward(leftSpeed, rightSpeed);
    } else {
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
  
  // Detect rising edge (transition from white to black)
  if (currentRightSensorState && !lastRightSensorState) {
    if (isDelivering) {
      currentTable++;
      Serial.print("currentTable: ");
      Serial.println(currentTable);
      
      // Check if we've reached the target table
      if (currentTable >= targetTable) {
        stopMotors();
        Serial.println("mode:returning");
        isDelivering = false;
        delay(2000); // Pause for delivery
      }
    } else {
      // Returning mode - decrement table count
      currentTable--;
      Serial.print("Returning... Current table: ");
      Serial.println(currentTable);
      
      // Check if we've returned to start
      if (currentTable <= 0) {
        stopMotors();
        Serial.println("mode:arrived");
        targetTable = 0; // Reset for next delivery
        Serial.println("Delivery complete! Ready for next order.");
      }
    }
  }
  
  lastRightSensorState = currentRightSensorState;
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

// ========== CALIBRATION FUNCTIONS ==========
void calibrateSensors() {
  // Optional: Add sensor calibration routine
  Serial.println("Calibrating sensors...");
  delay(3000);
  Serial.println("Calibration complete!");
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