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
 * @date August 24, 2022
 * 
 * @version v1.0
 */

#define SENSOR_SAMPLE_RATE 40 // Hz

// LCD Instantiation
#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 8, 9, 10, 11, 12); // RS, En, D4, D5, D6, D7

// MPU6050 Instantiation
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include <math.h>
#include "utility/quaternion.h"

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Kalman filter instantiation
#include <Kalman.h>

// IMU Kalman filter
#define N_state_imu 6 // Number of states - Acceleration (XYZ), Gyroscope (XYZ)
#define N_obs_imu 6 // Number of observations - Acceleration (XYZ), Gyroscope (XYZ)
#define r_a 0.1 // Acceleration measurement noise
#define r_g 0.1 // Gyroscope measurement noise
#define q_a 0.5 // Process error - acceleration
#define q_g 0.5 // Process error - gyroscope

BLA::Matrix<N_obs_imu> obs_imu; // Observation vector for the SVA filter
KALMAN<N_state_imu, N_obs_imu> K_imu; // Kalman filter object

// SVA Kalman filter
#define N_state_sva 9 // Number of states - Position (XYZ), Speed (XYZ), Linear Acceleration (XYZ)
#define N_obs_sva 3 // Number of observation - Linear Acceleration (XYZ)
#define q_a 0.5 // Process error - linear acceleration
#define q_p 0.1 // Process error - position
#define q_s 0.1 // Process error - speed
#define q_a 0.5 // Process error - acceleration

BLA::Matrix<N_obs_sva> obs_sva; // Observation vector for the SVA filter
KALMAN<N_state_sva, N_obs_sva> K_sva; // Kalman filter object
unsigned long currMillis;
float dt;

float sxOffset, syOffset, szOffset, vxOffset, vyOffset, vzOffset, hdgOffset = 0;

// Mahony Filter Instantiation
#include "MahonyAHRS.h"
Mahony M(SENSOR_SAMPLE_RATE, 10.0f, 0.1f); // Mahony filter

// General instantiations
#include <avr/sleep.h>

#define PAGE_CHANGE_BUTTON_PIN 2
#define RESET_BUTTON_PIN 3
#define RESET_HOLD_TIME 500 // ms
#define BUZZER_ACTIVE_PIN 4
#define BUZZER_ACTIVE_TIME 125 // ms
#define MAX_NUM_PAGES 6 // Acceleration, Lin. Accel., Speed, Displacement, Gyro, Orientation
#define ARDUINO_WAKE_PIN 18
#define ARDUINO_ACTIVE_TIME 10000 // ms

bool isBuzzerActive = false;
long buzzerStartTime = 0;
long resetStartTime = 0;
long wakeUpTime = 0;
uint8_t curPageNum = 0; // 0: acceleration, 1: gyroscope, 2: orientation
bool isPageChanged = false;
bool isResetTriggered = true;

struct telemetry_t {
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

    float linAccelX = 0;
    float linAccelY = 0;
    float linAccelZ = 0;
} data;



void setup() {
    Serial.begin(9600);
    // Pin initializations
    pinMode(BUZZER_ACTIVE_PIN, OUTPUT);
    pinMode(PAGE_CHANGE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ARDUINO_WAKE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PAGE_CHANGE_BUTTON_PIN), pageChangeISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetButtonISR, FALLING);

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

    mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
    Serial.print("Cycle Rate set to: ");
    switch (mpu.getCycleRate()) {
    case MPU6050_CYCLE_1_25_HZ:
        Serial.println("1.25 Hz");
        break;
    case MPU6050_CYCLE_5_HZ:
        Serial.println("5 Hz");
        break;
    case MPU6050_CYCLE_20_HZ:
        Serial.println("20 Hz");
        break;
    case MPU6050_CYCLE_40_HZ:
        Serial.println("40 Hz");
        break;
    }
    
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

    // mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    // Serial.print("Filter bandwidth set to: ");
    // switch (mpu.getFilterBandwidth()) {
    // case MPU6050_BAND_260_HZ:
    //     Serial.println("260 Hz");
    //     break;
    // case MPU6050_BAND_184_HZ:
    //     Serial.println("184 Hz");
    //     break;
    // case MPU6050_BAND_94_HZ:
    //     Serial.println("94 Hz");
    //     break;
    // case MPU6050_BAND_44_HZ:
    //     Serial.println("44 Hz");
    //     break;
    // case MPU6050_BAND_21_HZ:
    //     Serial.println("21 Hz");
    //     break;
    // case MPU6050_BAND_10_HZ:
    //     Serial.println("10 Hz");
    //     break;
    // case MPU6050_BAND_5_HZ:
    //     Serial.println("5 Hz");
    //     break;
    // }

    // Initialize IMU Kalman Filter
    // IMU time evolution matrix
    //          Ax | Ay | Az | Gx | Gy | Gz
    K_imu.F = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
    
    // IMU measurement matrix
    //          Ax | Ay | Az | Gx | Gy | Gz
    K_imu.H = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

    double ra2 = r_a*r_a;
    double rg2 = r_g*r_g;
    // IMU measurement covariance matrix
    //          Ax | Ay | Az | Gx | Gy | Gz
    K_imu.R = { ra2, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, ra2, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, ra2, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, rg2, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, rg2, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, rg2 };

    // IMU model covariance matrix
    double qa2 = q_a*q_a;
    double qg2 = q_g*q_g;
    K_imu.Q = { qa2, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, qa2, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, qa2, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, qg2, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, qg2, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, qg2 };
    
    // Initialize SVA Kalman Filter
    // SVA time evolution matrix
    //          P  | S  | lA | P  | S  | lA | P  | S  | lA
    K_sva.F = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // X
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Y
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // Z
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

    // SVA measurement matrix (position, velocity, linear acceleration)
    K_sva.H = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // X
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    // Y
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };  // Z
    
    // SVA measurement covariance matrix
    K_sva.R = { ra2, 0.0, 0.0,
                0.0, ra2, 0.0,
                0.0, 0.0, ra2 };
    
    // SVA model covariance matrix
    double qp2 = q_p*q_p;
    double qs2 = q_s*q_s;
    K_sva.Q = { qp2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // X
                0.0, qs2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, qa2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, qp2, 0.0, 0.0, 0.0, 0.0, 0.0, // Y
                0.0, 0.0, 0.0, 0.0, qs2, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, qa2, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qp2, 0.0, 0.0, // Z
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qs2, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qa2 };
    
    currMillis = millis();

    Serial.println();
    delay(100);
}

void loop() {
    if (millis() < wakeUpTime+ARDUINO_ACTIVE_TIME) {
        static long _lastIMUPoll = 0;
        if (millis() > _lastIMUPoll+(1000/SENSOR_SAMPLE_RATE)) {
            _lastIMUPoll = millis();

            // Poll IMU
            mpu.getEvent(&a, &g, &temp);

            // Compute time difference since last cycle
            dt = (millis() - currMillis)/1000.0;
            currMillis = millis();

            // ===============================
            // === FILTER IMU MEASUREMENTS ===
            // ===============================

            // Since the measurements do not have a time-dependent component, we can just update the filter with the current measurements
            obs_imu(0) = a.acceleration.x;
            obs_imu(1) = a.acceleration.y;
            obs_imu(2) = a.acceleration.z;
            obs_imu(3) = g.gyro.x;
            obs_imu(4) = g.gyro.y;
            obs_imu(5) = g.gyro.z;
            
            // Update IMU Kalman filter
            K_imu.update(obs_imu);

            // Update telemetry packet
            data.accelX = K_imu.x(0);
            data.accelY = K_imu.x(1);
            data.accelZ = K_imu.x(2);
            data.gyroX  = K_imu.x(3);
            data.gyroY  = K_imu.x(4);
            data.gyroZ  = K_imu.x(5);

            // ==========================
            // === RUN MAHONY AHRS FILTER
            // ==========================

            // Update Mahony filter
            M.updateIMU(data.gyroX, data.gyroY, data.gyroZ,
                        data.accelX, data.accelY, data.accelZ); // TODO: Filter needs tuning to decrease phase shift
            // M.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z,
            //             a.acceleration.x, a.acceleration.y, a.acceleration.z);

            // Orientation
            data.roll = M.getRoll();
            data.pitch = M.getPitch();
            data.yaw = M.getYaw();

            // DEBUG
            // Serial.print("Ax: "); Serial.print(data.accelX); Serial.println(",");
            // Serial.print("Ay: "); Serial.print(data.accelY); Serial.println(",");
            // Serial.print("Az: "); Serial.print(data.accelZ); Serial.println(",");
            // Serial.print("Gx: "); Serial.print(data.gyroX); Serial.println(",");
            // Serial.print("Gy: "); Serial.print(data.gyroY); Serial.println(",");
            // Serial.print("Gz: "); Serial.print(data.gyroZ); Serial.println(",");
            // Serial.print("Roll: ");     Serial.print(data.roll); Serial.println(",");
            // Serial.print("Pitch: ");    Serial.print(data.pitch); Serial.println(",");
            // Serial.print("Yaw: ");      Serial.print(data.yaw); // Serial.println(",");
            // Serial.println(); Serial.println();

            // =====================================
            // === CALCULATE LINEAR ACCELERATION ===
            // =====================================

            sensors_vec_t linAccel;
            imu::Vector<3> gravNED = {0, 0, 9.81}; // Gravitational acceleration in NED

            imu::Quaternion Qb = M.getQuaternion();
            imu::Quaternion Qg = Qb.invert() * imu::Quaternion(0, gravNED) * Qb; // Rotate gravity from NED reference frame to body

            // --- UPDATE TELEMETRY PACKET ---
            data.linAccelX = a.acceleration.x - Qg.x();
            data.linAccelY = a.acceleration.y - Qg.y();
            data.linAccelZ = a.acceleration.z - Qg.z();

            // =========================
            // === RUN KALMAN FILTER ===
            // =========================

            // Update state equation
            // Here we make use of the Taylor expansion on the (position,speed,acceleration)
            // position_{k+1}     = position_{k} + dt*speed_{k} + (dt*dt/2)*acceleration_{k}
            // speed_{k+1}        = speed_{k} + dt*acceleration_{k}
            // acceleration_{k+1} = acceleration_{k}
            K_sva.F = { 1.0, dt,  dt*dt/2,  0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
                        0.0, 1.0, dt,       0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
                        0.0, 0.0, 1.0,      0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,      1.0, dt,  dt*dt/2,  0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,      0.0, 1.0, dt,       0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,      0.0, 0.0, 1.0,      0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      1.0, dt,  dt*dt*2,
                        0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      0.0, 1.0, dt,
                        0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      0.0, 0.0, 1.0};

            // Use linear acceleration for better accuracy
            obs_sva(0) = data.linAccelX;
            obs_sva(1) = data.linAccelY;
            obs_sva(2) = data.linAccelZ;
            
            // Update Kalman filter
            K_sva.update(obs_sva);

            // --- UPDATE TELEMETRY PACKET ---

            // Displacement
            data.dispX = K_sva.x(0) - sxOffset;
            data.dispY = K_sva.x(3) - syOffset;
            data.dispZ = K_sva.x(6) - szOffset;

            // Velocity
            data.speedX = K_sva.x(1) - vxOffset;
            data.speedY = K_sva.x(4) - vyOffset;
            data.speedZ = K_sva.x(7) - vzOffset;
        }

        // =========================
        // === UPDATE LCD MODULE ===
        // =========================

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
                lcd.setCursor(0,0); // Set cursor to top row
                lcd.print("Ax   Ay   Az  g"); // Write headers
                lcd.setCursor(0,1); // Set cursor to bottom row
                lcd.print(data.accelX/9.81, 1); data.accelX > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.accelY/9.81, 1); data.accelY > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.accelZ/9.81, 1); data.accelZ > 0 ? lcd.print("  ") : lcd.print(" ");

                break;

            case 1: // Linear Acceleration page
                lcd.setCursor(0,0); // Set cursor to top row
                lcd.print("lAx  lAy  lAz  g"); // Write headers
                lcd.setCursor(0,1); // Set cursor to bottom row
                lcd.print(data.linAccelX/9.81, 1); data.linAccelX > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.linAccelY/9.81, 1); data.linAccelY > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.linAccelZ/9.81, 1); data.linAccelZ > 0 ? lcd.print("  ") : lcd.print(" ");

                break;
            
            case 2: // Velocity page
                lcd.setCursor(0,0); // Set cursor to top row
                lcd.print("Ux   Uy   Uz m/s"); // Write headers
                lcd.setCursor(0,1); // Set cursor to bottom row
                lcd.print(data.speedX, 1); data.speedX > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.speedY, 1); data.speedY > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.speedZ, 1); data.speedZ > 0 ? lcd.print("  ") : lcd.print(" ");

                break;

            case 3: // Displacement page
                lcd.setCursor(0,0); // Set cursor to top row
                lcd.print("Sx   Sy   Sz  m"); // Write headers
                lcd.setCursor(0,1); // Set cursor to bottom row
                lcd.print(data.dispX, 1); data.dispX > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.dispY, 1); data.dispY > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.dispZ, 1); data.dispZ > 0 ? lcd.print("  ") : lcd.print(" ");

                break;
            
            case 4: // Gyroscope page
                lcd.setCursor(0,0); // Set cursor to top row
                lcd.print("Gx   Gy   Gz r/s"); // Write headers
                lcd.setCursor(0,1); // Set cursor to bottom row
                lcd.print(data.gyroX, 1); data.gyroX > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.gyroY, 1); data.gyroY > 0 ? lcd.print("  ") : lcd.print(" ");
                lcd.print(data.gyroZ, 1); data.gyroZ > 0 ? lcd.print("  ") : lcd.print(" ");

                break;

            case 5: // Orientation page
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

        // Reset handler
        if (isResetTriggered && !digitalRead(RESET_BUTTON_PIN) && millis() >= resetStartTime + RESET_HOLD_TIME) { // Check for a button press and if the button is held for RESET_HOLD_TIME
            isResetTriggered = false; // Reset the reset button flag
            // Reset displacement values
            sxOffset += data.dispX; 
            syOffset += data.dispY;
            szOffset += data.dispZ;

            // Reset speed values
            vxOffset += data.speedX;
            vyOffset += data.speedY;
            vzOffset += data.speedZ;

            // Reset heading value
            hdgOffset += data.yaw;

            isBuzzerActive = true;
            buzzerStartTime = millis();
            Serial.println("Reset values!"); // DEBUG
        }

        // delay(1000/SENSOR_SAMPLE_RATE);
    }
    else {
        arduinoSleep();
    }
}


// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================


void pageChangeISR() {
    // buzzerStartTime = millis(); // Start the buzzer active timer
    // isBuzzerActive = true; // Set the buzzer active flag
    isPageChanged = true; // Set the page changed flag
    curPageNum++; // Change the page number
    if (curPageNum >= MAX_NUM_PAGES) { // Increment the page number and check if it is greater than the max pages
        curPageNum = 0; // Reset current page to first
    }
    // Serial.println(curPageNum); // DEBUG
}

void resetButtonISR() {
    isResetTriggered = true;
    resetStartTime = millis(); 
    Serial.println("Reset button pressed!"); // DEBUG
}

void wakeUpISR() {
    wakeUpTime = millis();
    Serial.println("Woken up!"); // DEBUG
    detachInterrupt(ARDUINO_WAKE_PIN);
}


// ==============================
// === SLEEP HANDLER ROUTINES ===
// ==============================


void arduinoSleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here  
    sleep_enable(); // enables the sleep bit in the mcucr register  
    delay(5);
    Serial.println("About to sleep");
    delay(5);
    attachInterrupt(digitalPinToInterrupt(ARDUINO_WAKE_PIN), wakeUpISR, LOW); // use interrupt 0 (pin 2) and run function  
    delay(5);
    Serial.println("Interrupt attached");
    delay(5);
    sleep_mode(); // here the device is actually put to sleep...!!

    // THE PROGRAM CONTINUES FROM HERE AFTER INTERRUPT IS CLOSED
    delay(5);
    Serial.println("Continuing main program after interrupt");
    delay(5);

    sleep_disable(); // first thing after waking from sleep: disable sl¯eep...  
    delay(5);
    Serial.println("Sleep disabled");
    delay(5);
}