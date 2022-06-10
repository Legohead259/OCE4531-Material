/**
 * OCE4531 Project 4: MPU6050 Accelerometer and LCD Display
 * @brief This is the solution for OCE4531's fourth and final project, the accelerometer display
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * This is the code for the undergraduate portion of the project
 * 
 * @author Braidan Duffy
 * 
 * @date May 30, 2022
 * 
 * @version v1.0
 */

// LCD Instantiation
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 9, 10, 11, 12); // RS, En, D4, D5, D6, D7

// MPU6050 Instantiation
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include <math.h>

Adafruit_MPU6050 mpu;

// General instantiations
#define PAGE_CHANGE_BUTTON_PIN 2
#define BUZZER_ACTIVE_PIN 6
#define BUZZER_ACTIVE_TIME 250 // ms
#define MAX_NUM_PAGES 3

bool isBuzzerActive = false;
long buzzerStartTime = 0;
uint8_t curPageNum = 0; // 0: acceleration, 1: gyroscope, 2: orientation
bool isPageChanged = false;

struct Telemetry {
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
    float roll = 0;
    float pitch = 0;
} data;

void setup() {
    Serial.begin(115200);
    // Pin initializations
    pinMode(BUZZER_ACTIVE_PIN, OUTPUT);
    pinMode(PAGE_CHANGE_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PAGE_CHANGE_BUTTON_PIN), pageChangeISR, FALLING);

    // Set up the LCD's number of columns and rows:
    Serial.print("Initializing LCD...");
    lcd.begin(16, 2);
    Serial.println("done!");

    Serial.print("Initializing IMU...");
    if (!mpu.begin()) { // Try to initialize!
        Serial.println("Failed to find MPU6050 chip");
        while (true); // Block further code execution
    }
    Serial.println("done!"); Serial.println();
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println();
    delay(100);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Wipe LCD screen on mode change
    if (isPageChanged) { // Check the page changed flag
        isPageChanged = false; // Reset the page changed flag
        lcd.setCursor(0, 0); // Set cursor to top row
        lcd.print("                "); // Clear top row
        lcd.setCursor(0, 1); // Set cursor to bottom row
        lcd.print("                "); // Clear bottom row
    }

    char _topRowBuffer[16]; // Buffer for LCD top row information
    char _bottomRowBuffer[16]; // Buffer for LCD bottom row information 
    switch (curPageNum) {
        case 0: // Acceleration page
            // Update telemetry package
            data.accelX = a.acceleration.x;
            data.accelY = a.acceleration.y;
            data.accelZ = a.acceleration.z;

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("Ax   Ay   Az"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.accelX/9.81, 1); data.accelX > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.accelY/9.81, 1); data.accelY > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.accelZ/9.81, 1); data.accelZ > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print("g");

            break;
        case 1: // Gyroscope page
            // Update telemetry package
            data.gyroX = g.gyro.x;
            data.gyroY = g.gyro.y;
            data.gyroZ = g.gyro.z;

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("Gx   Gy   Gz r/s"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.gyroX, 1); data.gyroX > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.gyroY, 1); data.gyroY > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.gyroZ, 1); data.gyroZ > 0 ? lcd.print("  ") : lcd.print(" ");

            break;

        case 2: // Orientation page
            // Update telemetry package
            data.accelX = a.acceleration.x;
            data.accelY = a.acceleration.y;
            data.accelZ = a.acceleration.z;

            data.pitch = 180 * atan (data.accelX/sqrt(data.accelY*data.accelY + data.accelZ*data.accelZ))/M_PI;
            data.roll = 180 * atan (data.accelY/sqrt(data.accelX*data.accelX + data.accelZ*data.accelZ))/M_PI;

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("R    P   [deg]"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.roll, 1); data.roll > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.pitch, 1); data.pitch > 0 ? lcd.print("  ") : lcd.print(" ");
            // lcd.print(data.yaw, 1); data.gyroZ > 0 ? lcd.print("  ") : lcd.print(" ");

            break;
    }

    if (isBuzzerActive) {
        if (millis() < buzzerStartTime+BUZZER_ACTIVE_TIME) {
            analogWrite(BUZZER_ACTIVE_PIN, 128); // Turn on buzzer
        }
        else {
            digitalWrite(BUZZER_ACTIVE_PIN, LOW);
            isBuzzerActive = false;
        }
    }

    delay(100);
}


// ========================
// === UPDATE FUNCTIONS ===
// ========================

// void updateDisplay(char* topRow, char* bottomRow) {
//     lcd.setCursor(0,0); // Set cursor to top row
//     lcd.print(topRow); // Write the top row data
//     lcd.setCursor(0,1); // Set cursor to bottom row
//     lcd.print(bottomRow); // Write the bottom row data
// }


// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================

void pageChangeISR() {
    buzzerStartTime = millis();
    isBuzzerActive = true; // Set the buzzer active flag
    curPageNum++;
    isPageChanged = true; // Set the page changed flag
    if (curPageNum >= MAX_NUM_PAGES) { // Increment the page number and check if it is greater than the max pages
        curPageNum = 0; // Reset current page to first
    }
    // Serial.println(curPageNum); // DEBUG
}