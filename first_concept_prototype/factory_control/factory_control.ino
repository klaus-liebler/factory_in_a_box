#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <AccelStepper.h>
#include <cppQueue.h>
#include <Servo.h>  // Added missing Servo library

#define STEP_PIN 6          //pin for the door of the slides
#define DIR_PIN 5           //pin for the door of the slides
#define STEP_PIN_EL 9       //pin for the elevator
#define DIR_PIN_EL 10       //pin for the elevator
#define BAND_PIN 8          //Pin to control conveyor
#define SERVO_PIN 11        //Pin for the servo of the elevator
#define MIN_DIFFERENCE 0.1  // Increased sensitivity threshold
#define DELAY_TIME 16000
#define DEBOUNCE_DELAY 1000
#define COLOR_STABILITY_TIME 500        // Time to confirm color stability
#define COLOR_DIFFERENCE_THRESHOLD 0.2  // Threshold for significant color change
#define MAX_ORDERS 10                   // Max number of orders in the queue

// Structure to store tower block information
struct TowerOrder {
  String blocks[3];  // Stores the blocks used
};

// Declare where the door needs to go based on color sensor input
const int redPosition = -1700;
const int greenPosition = 0;
const int bluePosition = 1700;

// Flag to check if the program is running
bool programRunning = true;

// Time tracking for color detection
unsigned long colorDetectionStartTime = 0;
char stableDetectedColor = '\0';

// Queue for storing color sensor reads
struct ColorData {
  char color;
  unsigned long time;
};
cppQueue queue(sizeof(ColorData), 10, FIFO);

// Define the two stepper motors and the servo
AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper elevator(AccelStepper::DRIVER, STEP_PIN_EL, DIR_PIN_EL);
Servo myServo;

// Define the color sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_4X);

// Factory storage counts
int redCount = 3;
int greenCount = 3;
int blueCount = 3;
int totalBlocks = redCount + greenCount + blueCount;

// Function to move elevator and activate servo
void moveToPositionAndActivateServo(int position, const char* message) {
  moveToPosition(position, message);

  // Activate the servo
  //Serial.println("Activating servo...");
  myServo.write(130);  // Move servo to 130°
  delay(1000);         // Keep servo at 130° for 1 second
  myServo.write(0);    // Return servo to 0°
  //Serial.println("Servo returned to position 0.");
}

// Function to move elevator to the given position
void moveToPosition(int position, const char* message) {
  //Serial.println(message);
  elevator.moveTo(position);

  // Move to desired position
  while (elevator.distanceToGo() != 0) {
    elevator.run();
  }
  //Serial.print("Position reached: ");
  //Serial.println(position);
}

// Function to send factory storage data to backend
void sendColorDataToBackend(char currentBlockColor = '\0') {
  String data = "{\"red\":";
  data += String(redCount);
  data += ",\"green\":";
  data += String(greenCount);
  data += ",\"blue\":";
  data += String(blueCount);
  data += ",\"total\":";
  data += String(totalBlocks);
  data += ",\"currentBlock\":\"";
  data += String(currentBlockColor);
  data += "\"}";
  Serial.println(data);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(BAND_PIN, OUTPUT);
  digitalWrite(BAND_PIN, LOW);

  if (tcs.begin()) {
    // Color sensor initialized successfully
  } else {
    // Error with color sensor
    while (1)
      ;
  }

  // Stepper motor configuration
  motor.setCurrentPosition(0);
  motor.setMaxSpeed(2000);
  motor.setAcceleration(1000);

  elevator.setCurrentPosition(0);
  elevator.setMaxSpeed(2000);
  elevator.setAcceleration(1000);

  // Servo setup
  myServo.attach(SERVO_PIN);  // Connect servo to pin 11
  myServo.write(0);           // Initialize servo at position 0°

  // System initialized
}

void loop() {
  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "START") {
      programRunning = true;
      digitalWrite(BAND_PIN, HIGH);  // Turn conveyor on
    } else if (command == "STOP") {
      programRunning = false;
      stopProgram();
    }
    if (command == "INIT") {
      // Move and activate servo in specified positions
      moveToPositionAndActivateServo(-610, "Moving to -580 steps...");
      delay(5000);
      moveToPositionAndActivateServo(-700, "Moving to -670 steps...");
      delay(5000);
      moveToPositionAndActivateServo(-760, "Moving to -750 steps...");
      delay(5000);

      // Return to initial position
      moveToPosition(0, "Returning to initial position...");
    }
    if (command.startsWith("TOWER")) {
      // Process tower block info
      String towerInfo = command.substring(6);  // Extract tower info
      int index1 = towerInfo.indexOf(',');
      int index2 = towerInfo.indexOf(',', index1 + 1);

      String block1 = towerInfo.substring(0, index1);
      String block2 = towerInfo.substring(index1 + 1, index2);
      String block3 = towerInfo.substring(index2 + 1);

      // Update inventory based on blocks used
      updateInventory(block1);
      updateInventory(block2);
      updateInventory(block3);

      totalBlocks -= 3;          // Subtract the 3 blocks from total
      sendColorDataToBackend();  // Send updated data to backend
    }
  }

  if (programRunning) {
    processColorDetection();
    processColorQueue();
  }
}

// Stop the program and reset conveyor
void stopProgram() {
  motor.moveTo(0);
  while (motor.distanceToGo() != 0) {
    motor.run();
  }
  digitalWrite(BAND_PIN, LOW);
  queue.flush();
}

// Detect color and update counts
void processColorDetection() {
  static char lastDetectedColor = '\0';
  static unsigned long lastDetectionTime = 0;

  uint16_t r, g, b, c;
  float redRatio, greenRatio, blueRatio;

  tcs.getRawData(&r, &g, &b, &c);

  // Prevent division by zero
  if (c == 0) return;

  redRatio = (float)r / (float)c;
  greenRatio = (float)g / (float)c;
  blueRatio = (float)b / (float)c;

  char currentDetectedColor = determineDominantColor(redRatio, greenRatio, blueRatio);

  unsigned long currentTime = millis();

  // New color detection logic
  if (currentDetectedColor != '\0') {
    if (currentDetectedColor != lastDetectedColor) {
      // Reset detection timer when color changes
      lastDetectionTime = currentTime;
      lastDetectedColor = currentDetectedColor;
    } else if (currentTime - lastDetectionTime >= COLOR_STABILITY_TIME) {
      // Color has been stable for enough time
      if (stableDetectedColor != currentDetectedColor) {
        // New stable color detected
        stableDetectedColor = currentDetectedColor;

        // Update block counts
        if (stableDetectedColor == 'R') {
          redCount++;
        } else if (stableDetectedColor == 'G') {
          greenCount++;
        } else if (stableDetectedColor == 'B') {
          blueCount++;
        }
        totalBlocks++;

        // Add to queue
        ColorData newColor = { stableDetectedColor, currentTime };
        queue.push(&newColor);
      }
    }
  }

  delay(100);  // Reduce detection frequency
}

// Determine the dominant color based on ratios
char determineDominantColor(float redRatio, float greenRatio, float blueRatio) {
  // More robust color determination
  float maxRatio = max(max(redRatio, greenRatio), blueRatio);
  float minRatio = min(min(redRatio, greenRatio), blueRatio);

  // Ensure significant color difference
  if (maxRatio - minRatio < COLOR_DIFFERENCE_THRESHOLD) {
    return '\0';  // Insufficient color distinction
  }

  if (maxRatio == redRatio && (maxRatio - greenRatio) >= MIN_DIFFERENCE && (maxRatio - blueRatio) >= MIN_DIFFERENCE) {
    return 'R';
  } else if (maxRatio == greenRatio && (maxRatio - redRatio) >= MIN_DIFFERENCE && (maxRatio - blueRatio) >= MIN_DIFFERENCE) {
    return 'G';
  } else if (maxRatio == blueRatio && (maxRatio - redRatio) >= MIN_DIFFERENCE && (maxRatio - greenRatio) >= MIN_DIFFERENCE) {
    return 'B';
  }

  return '\0';
}

// Process the color queue and move motors accordingly
void processColorQueue() {
  if (!queue.isEmpty()) {
    ColorData firstColor;
    queue.peek(&firstColor);
    if (millis() - firstColor.time >= DELAY_TIME) {
      char currentBlock = firstColor.color;
      sendColorDataToBackend(currentBlock);
      moveMotorToColorPosition(currentBlock);
      while (motor.distanceToGo() != 0) {
        motor.run();
      }
      queue.pop(&firstColor);
    }
  }
}

// Move motor based on detected color
void moveMotorToColorPosition(char color) {
  if (color == 'R') motor.moveTo(redPosition);
  else if (color == 'G') motor.moveTo(greenPosition);
  else if (color == 'B') motor.moveTo(bluePosition);
}

// Update inventory count based on color
void updateInventory(String block) {
  if (block == "R") redCount--;
  else if (block == "G") greenCount--;
  else if (block == "B") blueCount--;
}

void updateInventoryplus(String block) {
  if (block == "R") redCount++;
  else if (block == "G") greenCount++;
  else if (block == "B") blueCount++;
}
