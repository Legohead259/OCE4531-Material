/**
 * OCE4531 Project 4: MPU6050 Accelerometer and LCD Display
 * @brief This is the solution for OCE4531's fourth and final project, the accelerometer display
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * 
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
#define RESET_BUTTON_PIN 3
#define RESET_HOLD_TIME 500 // ms
#define BUZZER_ACTIVE_PIN 6
#define BUZZER_ACTIVE_TIME 250 // ms
#define MAX_NUM_PAGES 5

bool isBuzzerActive = false;
long buzzerStartTime = 0;
long resetStartTime = 0;
uint8_t curPageNum = 0; // 0: acceleration, 1: gyroscope, 2: orientation
bool isPageChanged = false;
bool isResetTriggered = true;

struct Telemetry {
    long timestamp = 0;
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;
    float roll = 0;
    float pitch = 0;
    
    // Reference: https://forum.arduino.cc/t/integration-of-acceleration/158296/14
    float yaw = 0;
    float speedX = 0;
    float speedY = 0;
    float speedZ = 0;
    float dispX = 0;
    float dispY = 0;
    float dispZ = 0;
} data;

void setup() {
    Serial.begin(115200);
    // Pin initializations
    pinMode(BUZZER_ACTIVE_PIN, OUTPUT);
    pinMode(PAGE_CHANGE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PAGE_CHANGE_BUTTON_PIN), pageChangeISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetButtonISR, CHANGE);

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

    switch (curPageNum) {
        case 0: // Acceleration page
            // Update telemetry package
            data.timestamp = millis();
            data.accelX = a.acceleration.x;
            data.accelY = a.acceleration.y;
            data.accelZ = a.acceleration.z;

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("Ax   Ay   Az  g"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.accelX/9.81, 1); data.accelX > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.accelY/9.81, 1); data.accelY > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.accelZ/9.81, 1); data.accelZ > 0 ? lcd.print("  ") : lcd.print(" ");

            break;
        
        case 1: // Velocity page
            updateVelocity(a);

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("Ux   Uy   Uz m/s"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.speedX, 1); data.speedX > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.speedY, 1); data.speedY > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.speedZ, 1); data.speedZ > 0 ? lcd.print("  ") : lcd.print(" ");

            break;

        case 2: // Displacement page
            updateDisplacement(a);

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("Sx   Sy   Sz  m"); // Write headers
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.dispX, 1); data.dispX > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.dispY, 1); data.dispY > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.dispZ, 1); data.dispZ > 0 ? lcd.print("  ") : lcd.print(" ");

            break;
        
        case 3: // Gyroscope page
            // Update telemetry package
            data.timestamp = millis();
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

        case 4: // Orientation page
            // Update telemetry package
            data.accelX = a.acceleration.x;
            data.accelY = a.acceleration.y;
            data.accelZ = a.acceleration.z;

            long _oldTimestamp = data.timestamp;
            float _oldGyroZ = data.gyroZ;

            // Update telemetry packet
            data.gyroZ = g.gyro.z;
            data.timestamp = millis();
            long dt = millis() - _oldTimestamp;
            float _avgGyroZ = (_oldGyroZ + data.gyroZ) / 2.0;

            data.yaw = data.yaw + _avgGyroZ*dt; // Calculate heading by integrating the gyro rotation about the Z axis
            data.pitch = 180 * atan (data.accelX/sqrt(data.accelY*data.accelY + data.accelZ*data.accelZ))/M_PI;
            data.roll = 180 * atan (data.accelY/sqrt(data.accelX*data.accelX + data.accelZ*data.accelZ))/M_PI;

            // Update LCD display
            lcd.setCursor(0,0); // Set cursor to top row
            lcd.print("R    P    Y deg"); // Write headers, 0xDF is the code for °
            lcd.setCursor(0,1); // Set cursor to bottom row
            lcd.print(data.roll, 1); data.roll > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.pitch, 1); data.pitch > 0 ? lcd.print("  ") : lcd.print(" ");
            lcd.print(data.yaw, 1); data.yaw > 0 ? lcd.print("  ") : lcd.print(" ");

            break;
    }

    // Buzzer active handler
    if (isBuzzerActive) {
        if (millis() < buzzerStartTime+BUZZER_ACTIVE_TIME) {
            analogWrite(BUZZER_ACTIVE_PIN, 128); // Turn on buzzer
        }
        else {
            digitalWrite(BUZZER_ACTIVE_PIN, LOW);
            isBuzzerActive = false;
        }
    }

    // Integral reset handler
    if (isResetTriggered && !digitalRead(RESET_BUTTON_PIN) && millis() >= resetStartTime + RESET_HOLD_TIME) { // Check for a button press and if the button is held for RESET_HOLD_TIME
        isResetTriggered = false; // Reset the reset button flag
        // Reset speed values
        data.speedX = 0; 
        data.speedY = 0;
        data.speedZ = 0;

        // Reset displacement values
        data.dispX = 0;
        data.dispY = 0;
        data.dispZ = 0;

        // Reset heading value
        data.yaw = 0;
        // Serial.println("Reset values!"); // DEBUG
    }

    delay(100);
}


// ========================
// === UPDATE FUNCTIONS ===
// ========================


void updateVelocity(sensors_event_t& a) {
    // Get previous time, acceleration, and speed values
    long _oldTimestamp = data.timestamp;
    float _oldAccels[3] = {data.accelX, data.accelY, data.accelZ};
    float _oldSpeeds[3] = {data.speedX, data.speedY, data.speedZ};

        // Update telemetry package
    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;
    data.timestamp = millis();

    float _newAccels[3] = {data.accelX, data.accelY, data.accelZ};
    long dt = millis() - _oldTimestamp;
    float _newSpeeds[3];

    for (int i=0; i<3; i++) { // For every axis (0=X, 1=Y, 2=Z)
        float _avgAccel = (_oldAccels[i] + _newAccels[i]) / 2.0;            
        _newSpeeds[i] = _oldSpeeds[i] + _avgAccel*dt; // Calculate the new speeds

        // Serial.print(_avgAccel); Serial.print(", "); // DEBUG
        Serial.print(_oldSpeeds[i]); Serial.print(", "); // DEBUG
        // Serial.print(_newSpeeds[i], 1); Serial.print(", "); // DEBUG
    }
    Serial.println(); // DEBUG

    data.speedX = _newSpeeds[0];
    data.speedY = _newSpeeds[1];
    data.speedZ = _newSpeeds[2];
}

void updateDisplacement(sensors_event_t& a) {
    // Get previous time, acceleration, and speed values
    long _oldTimestamp = data.timestamp;
    float _oldAccels[3] = {data.accelX, data.accelY, data.accelZ};
    float _oldSpeeds[3] = {data.speedX, data.speedY, data.speedZ};
    float _oldDisps[3] = {data.dispX, data.dispY, data.dispZ};

    // Update telemetry package
    data.accelX = a.acceleration.x;
    data.accelY = a.acceleration.y;
    data.accelZ = a.acceleration.z;
    data.timestamp = millis();

    float _newAccels[3] = {data.accelX, data.accelY, data.accelZ};
    long dt = millis() - _oldTimestamp;
    float _newSpeeds[3];
    float _newDisps[3];

    for (int i=0; i<3; i++) { // For every axis (0=X, 1=Y, 2=Z)
        float _avgAccel = (_oldAccels[i] + _newAccels[i]) / 2.0;            
        _newSpeeds[i] = _oldSpeeds[i] + _avgAccel*dt; // Calculate the new speeds
        float _avgSpeed = (_oldSpeeds[i] + _newSpeeds[i]) / 2.0;
        _newDisps[i] = _oldDisps[i] + _avgSpeed*dt + _avgAccel*dt*dt; // Calculate the new displacements

        // Serial.print(_avgAccel); Serial.print(", "); // DEBUG
        // Serial.print(_oldSpeeds[i]); Serial.print(", "); // DEBUG
        // Serial.print(_newSpeeds[i], 1); Serial.print(", "); // DEBUG
    }
    Serial.println(); // DEBUG

    data.dispX = _newDisps[0];
    data.dispY = _newDisps[1];
    data.dispZ = _newDisps[2];
}


// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================


void pageChangeISR() {
    buzzerStartTime = millis(); // Start the buzzer active timer
    isBuzzerActive = true; // Set the buzzer active flag
    isPageChanged = true; // Set the page changed flag
    curPageNum++; // Change the page number
    if (curPageNum >= MAX_NUM_PAGES) { // Increment the page number and check if it is greater than the max pages
        curPageNum = 0; // Reset current page to first
    }
    // Serial.println(curPageNum); // DEBUG
}

void resetButtonISR() {
    if (digitalRead(RESET_BUTTON_PIN)) { // Button was released
        isResetTriggered = false; // Reset the reset button flag
        // Serial.println("Set reset trigger to FALSE"); // DEBUG
    }
    else { // Button was pressed
        resetStartTime = millis();
        isResetTriggered = true; // Set the reset button flag
        // Serial.println("Set reset trigger to TRUE"); // DEBUG
    }
}