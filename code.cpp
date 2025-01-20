#include <Arduino.h>
#include <Wire.h>
#include <limits.h>

// EEPROM constants
const int eepromAddress = 0x50;

// Maze constants
const int mazeSize = 16;
const int destinationX = 4;
const int destinationY = 4;

// Ultrasonic sensor pins
const int trigPin = 2;
const int echoPinFront = 6;
const int echoPinRight = 5;
const int echoPinLeft = 9;

// Motor driver pins
const int motorPin1 = 3;  
const int motorPin2 = 4;
const int motorPin3 = 7;
const int motorPin4 = 8;

// Movement delays
const int moveDelay = 300;
const int turnDelay = 500;

// Distance thresholds
const int frontThreshold = 12; // cm
const int sideThreshold = 15;  // cm

// Arrays for pathfinding
int distances[mazeSize][mazeSize];
int direction[mazeSize][mazeSize][2]; // Stores movement directions

// Current position
int currentRow = 0;
int currentCol = 0;

// Function to get distance from ultrasonic sensors
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  return pulseIn(echoPin, HIGH) * 0.0343 / 2.0; // Distance in cm
}

// Function to write data to EEPROM
void writeToEEPROM(int addr, int data) {
  Wire.beginTransmission(eepromAddress);
  Wire.write((byte)(addr >> 8));
  Wire.write((byte)(addr & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

// Function to read data from EEPROM
int readFromEEPROM(int addr) {
  int data = 0;
  Wire.beginTransmission(eepromAddress);
  Wire.write((byte)(addr >> 8));
  Wire.write((byte)(addr & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(eepromAddress, 1);
  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}

// Function to save the path to EEPROM
void savePathToEEPROM() {
  int index = 0;
  for (int row = 0; row < mazeSize; row++) {
    for (int col = 0; col < mazeSize; col++) {
      if (distances[row][col] < INT_MAX) { // Only save valid path data
        writeToEEPROM(index++, distances[row][col]);
        writeToEEPROM(index++, direction[row][col][0]);
        writeToEEPROM(index++, direction[row][col][1]);
      }
    }
  }
}

// Function to read the path from EEPROM
void readPathFromEEPROM() {
  int index = 0;
  for (int row = 0; row < mazeSize; row++) {
    for (int col = 0; col < mazeSize; col++) {
      distances[row][col] = readFromEEPROM(index++);
      direction[row][col][0] = readFromEEPROM(index++);
      direction[row][col][1] = readFromEEPROM(index++);
    }
  }
}

// Function to update distances for Dijkstra's algorithm
void updateDistances(int row, int col) {
  int currentDistance = distances[row][col];
  if (row > 0 && distances[row - 1][col] > currentDistance + 1) {
    distances[row - 1][col] = currentDistance + 1;
    direction[row - 1][col][0] = -1; // Up
    direction[row - 1][col][1] = 0;
  }
  if (row < mazeSize - 1 && distances[row + 1][col] > currentDistance + 1) {
    distances[row + 1][col] = currentDistance + 1;
    direction[row + 1][col][0] = 1; // Down
    direction[row + 1][col][1] = 0;
  }
  if (col > 0 && distances[row][col - 1] > currentDistance + 1) {
    distances[row][col - 1] = currentDistance + 1;
    direction[row][col - 1][0] = 0;
    direction[row][col - 1][1] = -1; // Left
  }
  if (col < mazeSize - 1 && distances[row][col + 1] > currentDistance + 1) {
    distances[row][col + 1] = currentDistance + 1;
    direction[row][col + 1][0] = 0;
    direction[row][col + 1][1] = 1; // Right
  }
}

// Function to implement Dijkstra's algorithm
void dijkstra() {
  for (int row = 0; row < mazeSize; row++) {
    for (int col = 0; col < mazeSize; col++) {
      distances[row][col] = INT_MAX;
      direction[row][col][0] = 0;
      direction[row][col][1] = 0;
    }
  }
  distances[currentRow][currentCol] = 0;

  while (true) {
    updateDistances(currentRow, currentCol);

    if (currentRow == destinationX && currentCol == destinationY) {
      break; // Destination reached
    }
  }
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(moveDelay);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(turnDelay);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(turnDelay);
}

// Main maze exploration function
void exploreMaze() {
  dijkstra();
  while (currentRow != destinationX || currentCol != destinationY) {
    float frontDistance = getDistance(trigPin, echoPinFront);
    float rightDistance = getDistance(trigPin, echoPinRight);
    float leftDistance = getDistance(trigPin, echoPinLeft);

    if (frontDistance > frontThreshold) {
      moveForward();
    } else if (rightDistance > sideThreshold) {
      turnRight();
    } else if (leftDistance > sideThreshold) {
      turnLeft();
    } else {
      Serial.println("Dead end detected!");
      break;
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(echoPinLeft, INPUT);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  Wire.begin();
}

void loop() {
  exploreMaze();
  savePathToEEPROM();
  delay(10000); // Pause before restarting
}
