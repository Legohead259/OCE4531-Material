/**
 * OCE4531 Project 3: 4-Digit 7-Segment Display Counter
 * @brief This is the solution for OCE4531's third project, 7-segment counter
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * This is the code for the graduate portionof the project
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
#define ABSOLUTE_MAX_VALUE 9999 // 9999 for DEC, FFFF for Hex
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
#define DECREMENT_BUTTON_PIN 2  // Button to decrease the counter value
#define INCREMENT_BUTTON_PIN 3  // Button to increase the counter value
#define DELTA_1_BUTTON_PIN 4    // Changes the counter value by [DELTA_1]
#define DELTA_2_BUTTON_PIN 5    // Changes the counter value by [DELTA_2]
#define DELTA_3_BUTTON_PIN 6    // Changes the counter value by [DELTA_3]
#define RESET_HOLD_TIME 500     // Time the decrement button must be held to reset the counter [ms]
long decBtnPressTime = 0;       // Time the decrement button was pressed
int decButtonPresses = 0;

// Buzzer variables
#define BUZZER_PIN 14
#define BUZZER_ACTIVE_TIME 125 // ms
long buzzerStartTime = 0;
bool isBuzzerActive = false;

// Global variables
#define DELTA_1 10
#define DELTA_2 100
#define DELTA_3 1000
long startTime = millis();
long counter = 0;

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
    pinMode(DECREMENT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(INCREMENT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DELTA_1_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DELTA_2_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DELTA_3_BUTTON_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(DECREMENT_BUTTON_PIN), decrementCounterISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(INCREMENT_BUTTON_PIN), incrementCounterISR, FALLING);

    // Buzzer pin setup
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW); // Turn buzzer off
}

void loop() {
    updateDisplay(); // Always want to update the display to keep lit

    // Decrement button hold handler
    static long _oldButtonPresses = decButtonPresses;
    if (decButtonPresses != _oldButtonPresses && !digitalRead(DECREMENT_BUTTON_PIN) && mills() >= decBtnPressTime + RESET_HOLD_TIME) {// Check for a button press and that the button is held for RESET_PRESS_TIME
        _oldButtonPresses = decButtonPresses; // Reset button press counter
        counter = 0; // Reset global counter
    }

    // Buzzer handler
    static long _prevMillis = millis();
    long _curMillis = millis();
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
    // Break down counter to constituent digits
    int _num1000s = counter / 1000;
    int _rem1000s = counter % 1000;
    int _num100s = _rem1000s / 100;
    int _rem100s = _rem1000s % 100;
    int _num10s = _rem100s / 10;
    int _rem10s = _rem100s % 10;
    int _num1s = _rem10s;

    displayDigits[3] = _num1000s;
    displayDigits[2] = _num100s;
    displayDigits[1] = _num10s;
    displayDigits[0] = _num1s;

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

// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================

void decrementCounterISR() {
    decButtonPresses++; // Increment the number of times the decrememnt button has been pressed
    decBtnPressTime = millis(); // Start the countdown for the decrement button press time
    isBuzzerActive = true; // Enable the buzzer active flag
    buzzerStartTime = millis(); // Start the countdown for the buzzer active

    if (!digitalRead(DELTA_3_BUTTON_PIN)) // Delta 3 button pressed 
        counter -= DELTA_3;
    else if (!digitalRead(DELTA_2_BUTTON_PIN)) // Delta 2 button pressed
        counter -= DELTA_2;
    else if (!digitalRead(DELTA_1_BUTTON_PIN)) // Delta 1 button pressed
        counter -= DELTA_1;
    else // No delta specified
        counter--;

    if (counter < 0) // Check for negative and make 0
        counter = 0;
}   

void incrementCounterISR() {
    isBuzzerActive = true; // Enable the buzzer active flag
    buzzerStartTime = millis(); // Start the countdown for the buzzer active

    if (!digitalRead(DELTA_3_BUTTON_PIN)) // Delta 3 button pressed 
        counter += DELTA_3;
    else if (!digitalRead(DELTA_2_BUTTON_PIN)) // Delta 2 button pressed
        counter += DELTA_2;
    else if (!digitalRead(DELTA_1_BUTTON_PIN)) // Delta 1 button pressed
        counter += DELTA_1;
    else // No delta specified
        counter++;

    if (counter > ABSOLUTE_MAX_VALUE) // Check for number over absolute max and trim it
        counter = ABSOLUTE_MAX_VALUE;
}