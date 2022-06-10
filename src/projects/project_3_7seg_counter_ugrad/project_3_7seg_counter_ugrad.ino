/**
 * OCE4531 Project 3: 4-Digit 7-Segment Display Counter
 * @brief This is the solution for OCE4531's third project, 7-segment counter
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * This is the code for the undergraduate portion of the project
 * 
 * @author Braidan Duffy
 * 
 * @date May 26, 2022
 * 
 * @version v1.0
 */

// 7-segment display pins
#define DATA_PIN 8 // 74HC595 pin 8 DS
#define LATCH_PIN 9 // 74HC595 pin 9 STCP
#define CLOCK_PIN 10 // 74HC595 pin 10 SHCP
uint8_t displayMode = 0; // 0: Decimal, 1: Hex, 2: ?
const uint8_t DIGIT_CONTROL_PINS[] = {22, 23, 24, 25}; // MSB -> LSB (MSBFIRST)
byte displayDigits[] = {0, 0, 0, 0, 0}; // Digit 4 (LSB), Digit 3, Digit 2, Digit 1 (MSB)

const byte TABLE[] = { 
    0x3f, // 0
    0x06, // 1
    0x5b, // 2
    0x4f, // 3
    0x66, // 4
    0x6d, // 5
    0x7d, // 6
    0x07, // 7
    0x7f, // 8 
    0x6f, // 9
    0x77, // A
    0x7c, // b
    0x39, // C
    0x5e, // d
    0x79, // E
    0x71, // F
    0x00 // Blank
};

// Digital input pins
#define COUNTER_RESET_BUTTON_PIN 2 // Button to reset the counter value

// Buzzer variables
#define BUZZER_PIN 14
#define BUZZER_ACTIVE_TIME 125 // ms
long buzzerStartTime = 0;
bool isBuzzerActive = false;

// Global variables
long startTime = millis();

void setup() {
    // 7-segment pin setup
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);
    for (int x=0; x<4; x++){
        pinMode(DIGIT_CONTROL_PINS[x], OUTPUT);
        digitalWrite(DIGIT_CONTROL_PINS[x], HIGH);  // Turns off the digits  
    }

    // Digital input pin setup
    pinMode(COUNTER_RESET_BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(COUNTER_RESET_BUTTON_PIN), counterResetISR, FALLING);

    // Buzzer pin setup
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Turn buzzer off
}

void loop() {
    updateDisplay(); // Always want to update the display to keep lit
    
    static long _prevMillis = millis();
    long _curMillis = millis();

    if (_curMillis > _prevMillis+1000) { // Execute every 1000 ms or so
        _prevMillis = _curMillis; // Reset timing flag 
        updateCount();
    }

    if (isBuzzerActive) { // Button pressed, so buzzer must be active
        if (_curMillis < buzzerStartTime+BUZZER_ACTIVE_TIME) // Allotted buzzer on time has not passed
            analogWrite(BUZZER_PIN, 128); // Turn on buzzer
        else {
            isBuzzerActive = false;
            digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
        }
    }
}

// =========================
// === DISPLAY FUNCTIONS ===
// =========================

void turnOffDisplay() {
    for (int x=0; x<4; x++) {
        digitalWrite(DIGIT_CONTROL_PINS[x], HIGH);
    }
}

void display(int num) {
    digitalWrite(LATCH_PIN, LOW); // Enable 74HC595 data transfer
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, TABLE[num]);
    digitalWrite(LATCH_PIN, HIGH); // Disable 74HC595 data transfer
}

void updateDisplay() {
    for (int d=0; d<4; d++) { // For every digit, update value
        turnOffDisplay(); // Turn all digits off
        display(displayDigits[d]); // Update register value
        digitalWrite(DIGIT_CONTROL_PINS[d], LOW); // Turn on specific display
        delay(1);
        // Serial.print(displayDigits[x]); Serial.print(",");
    }
    // Serial.println();

    turnOffDisplay();
}

void updateCount() {
    if (displayDigits[0]==15 && displayDigits[1]==15 && displayDigits[2]==15 && displayDigits[3]==15) // If all digits are at a maximum value
        for (int d=0; d<4; d++) displayDigits[d] = 0; // reset display
    
    bool incrementValue = true;
    for (int d = 0; d < 4; d++){
        int x = int(displayDigits[d]);
        if (incrementValue == true) {
            x++;
            incrementValue = false;
            if (x > 9) {
                displayDigits[d] = 0;
                incrementValue = true;
            } else {
                displayDigits[d] = byte(x);
            }
        }
    }
}

// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================

void counterResetISR() {
    for (int d=0; d<4; d++) displayDigits[d] = 0; // reset display
    isBuzzerActive = true;
    buzzerStartTime = millis();
}