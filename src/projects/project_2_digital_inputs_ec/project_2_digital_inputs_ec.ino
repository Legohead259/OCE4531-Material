/**
 * OCE4531 Project 2: Digital Inputs and Interrupts Solution w/ Extra-Credit
 * @brief This is the solution for OCE4531's second project, Digital Inputs and Interrupts.
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * @author Braidan Duffy
 * 
 * @date September 04, 2022
 * 
 * @version v1.0
 */

// Define button pins according to schematic
#define HWREG_BUTTON_PIN 18 // Button that is regulated by a RC low-pass filter
#define SWREG_BUTTON_PIN 2 // Button that is regulated by a software debounce
#define UNREG_BUTTON_PIN 3 // Button that has an unregulated input

// Define LED pins according to schematic
#define HWREG_LED_PIN 10
#define SWREG_LED_PIN 11
#define UNREG_LED_PIN 12

// Define button hold time to trigger LED state change
#define BUTTON_HOLD_TIME 500 // ms

// Initialize LED states
bool hwregLEDState = false;
bool swregLEDState = false;
bool unregLEDState = false;

// Initialize button counters
long hwregBtnPresses = 0;
long swregBtnPresses = 0;
long unregBtnPresses = 0;

// Initialize button timers
long hwregBtnPressTime = 0;
long swregBtnPressTime = 0;
long unregBtnPressTime = 0;

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for Serial connection

    // Set button pins to inputs with internal pullup resistors
    pinMode(HWREG_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SWREG_BUTTON_PIN, INPUT_PULLUP);
    pinMode(UNREG_BUTTON_PIN, INPUT_PULLUP);

    // Attach Interrupt Service Routines (ISRs) to pins
    attachInterrupt(digitalPinToInterrupt(HWREG_BUTTON_PIN), hwregBtnISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(SWREG_BUTTON_PIN), swregBtnISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(UNREG_BUTTON_PIN), unregBtnISR, FALLING);

    // Set LED pins to outputs
    pinMode(HWREG_LED_PIN, OUTPUT);
    pinMode(SWREG_LED_PIN, OUTPUT);
    pinMode(UNREG_LED_PIN, OUTPUT);
}

void loop() {
    // Hardware-regulated button LED code
    // Button hold handler
    static long _oldHWPresses = 0;
    if (hwregBtnPresses != _oldHWPresses && !digitalRead(HWREG_BUTTON_PIN) && millis() > hwregBtnPressTime+BUTTON_HOLD_TIME) { // Check for button press, button currently being held, button held for BUTTON_HOLD_TIME
        hwregLEDState = !hwregLEDState; // Change LED state
        digitalWrite(HWREG_LED_PIN, hwregLEDState);
        _oldHWPresses = hwregBtnPresses; // Reset check counter
    }

    // Software-regulated button LED code
    // Button hold handler
    static long _oldSWPresses = 0;
    if (swregBtnPresses != _oldSWPresses && !digitalRead(SWREG_BUTTON_PIN) && millis() > swregBtnPressTime+BUTTON_HOLD_TIME) { // Check for button press, button currently being held, button held for BUTTON_HOLD_TIME
        swregLEDState = !swregLEDState; // Change LED state
        digitalWrite(SWREG_LED_PIN, swregLEDState);
        _oldSWPresses = swregBtnPresses; // Reset check counter
    }

    // Un-regulated button LED code
    // Button hold handler
    static long _oldUNPresses = 0;
    if (unregBtnPresses != _oldUNPresses && !digitalRead(UNREG_BUTTON_PIN) && millis() > unregBtnPressTime+BUTTON_HOLD_TIME) { // Check for button press, button currently being held, button held for BUTTON_HOLD_TIME
        unregLEDState = !unregLEDState; // Change LED state
        digitalWrite(UNREG_LED_PIN, unregLEDState);
        _oldUNPresses = unregBtnPresses; // Reset check counter
    }
}

// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================

void hwregBtnISR() {
    hwregBtnPresses++; // Increment button presses
    hwregBtnPressTime = millis(); // Record the current time when the button is pressed
    Serial.println("Hardware-regulated button pressed!");
    Serial.print("It has been pressed "); Serial.print(hwregBtnPresses); Serial.println(" times!");
    Serial.println();
}

void swregBtnISR() {
    static unsigned long _lastInterruptTime = 0;
    unsigned long _curInterruptTime = millis();
    if (_curInterruptTime - _lastInterruptTime > 100) { // If interrupts come faster than 100ms, assume it's a bounce and ignore
        swregBtnPresses++; // Increment button presses
        swregBtnPressTime = millis(); // Record the current time when the button is pressed
        Serial.println("Software-regulated button pressed!");
        Serial.print("It has been pressed "); Serial.print(swregBtnPresses); Serial.println(" times!");
        Serial.println();
    }
    _lastInterruptTime = _curInterruptTime;
}

void unregBtnISR() {
    unregBtnPresses++; // Increment button presses
    unregBtnPressTime = millis(); // Record the current time when the button is pressed
    Serial.println("Un-regulated button pressed!");
    Serial.print("It has been pressed "); Serial.print(unregBtnPresses); Serial.println(" times!");
    Serial.println();
}