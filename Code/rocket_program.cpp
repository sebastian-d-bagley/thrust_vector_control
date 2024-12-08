#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

// File open
File flightLog;

// PID Values
float kP = 0.3;
float kD = 0.1;

// Regression values for direct angle control (calibrated 10/18/24)
float mX = 4.878;
float bX = 72;

float mY = -3.962;
float bY = 52;

// Servo initialization
Servo xAxis;
Servo yAxis;

// Gyroscope calibration (calibrated 10/18/24)
float xOffset = 0.75;
float yOffset = 1;

// Variables for PID
float currentXGyro = 0;
float currentYGyro = 0;

float derivativeX = 0;
float derivativeY = 0;

float last_value_x = 0;
unsigned long last_micros_x = 0;

float last_value_y = 0;
unsigned long last_micros_y = 0;

// Inititalizing the gyroscope
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int chipSelect = 10;

unsigned long lastFlush = 0;


void setup() {
  // Connect the servos
  xAxis.attach(6);
  yAxis.attach(9);

  if (!bno.begin()) {
    while (1) {}
  }

  // Initialize the gyroscope
  bno.setExtCrystalUse(true);

  // Make sure the SD intializes
  if (!SD.begin(chipSelect)) {
    while (1) {}
  }

  // Open the log txt file
  flightLog = SD.open("log.txt", FILE_WRITE);

  // Ensure the log opens
  if (!flightLog) {
    while (1) {}
  }

  // Write intiial message to the SD card
  flightLog.println("\n\n\nFlight log for TVC 7.0");
  flightLog.println("Designed, programmed, and built by Sebastian Bagley");
  flightLog.println("-------------------------");
  flightLog.println("P: " + String(kP, 6) + "\tD: " + String(kD, 6));
  flightLog.println("Regression Values:");
  flightLog.println(" -X | m: " + String(mX, 6) + " b: " + String(bX, 6));
  flightLog.println(" -Y | m: " + String(mY, 6) + " b: " + String(bY, 6));
  flightLog.println("Post-flight notes: ");
  flightLog.println("-------------------------\n");
  flightLog.println("DATA BEGIN");
}

void loop() {
  // Get BNO055 sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  // "P" in PID
  currentXGyro = event.orientation.y + xOffset;
  currentYGyro = event.orientation.z + yOffset;

  if (last_value_x != currentXGyro) {
    derivativeX = (currentXGyro - last_value_x) / ((micros() - last_micros_x) / 1000000.0);
    last_micros_x = micros();
    last_value_x = currentXGyro;
  }
  if (last_value_y != currentYGyro) {
    derivativeY = (currentYGyro - last_value_y) / ((micros() - last_micros_y) / 1000000.0);
    last_micros_y = micros();
    last_value_y = currentYGyro;
  }

  float PID_X = currentXGyro * kP + derivativeX * kD;
  float PID_Y = currentYGyro * kP + derivativeY * kD;

  // Writing the PID values to the servos
  xAxis.write(angle_to_servo(PID_X, mX, bX));
  yAxis.write(angle_to_servo(-PID_Y, mY, bY));
  
  flightLog.print(micros());
  flightLog.print("\t");      
  flightLog.print(currentXGyro);
  flightLog.print("\t");
  flightLog.print(currentYGyro);
  flightLog.print("\n");

  if (millis() - lastFlush >= 1000) {
    flightLog.flush();
    lastFlush = millis();
  }
}

float angle_to_servo(float x, float m, float b) {
  return m * x + b;
}
