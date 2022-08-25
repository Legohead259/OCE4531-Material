/**
 * OCE4531 Project 2: Digital Inputs and Interrupts Solution
 * @brief This is the solution for OCE4531's second project, Digital Inputs and Interrupts.
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within.
 * See handout for more information
 * 
 * @author Braidan Duffy
 * 
 * @date May 25, 2022
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

// Initialize LED states
bool hwregLEDState = false;
bool swregLEDState = false;
bool unregLEDState = false;

// Initialize counters
long hwregBtnPresses = 0;
long swregBtnPresses = 0;
long unregBtnPresses = 0;

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
    digitalWrite(HWREG_LED_PIN, hwregLEDState);

    // Software-regulated button LED code
    digitalWrite(SWREG_LED_PIN, swregLEDState);

    // Un-regulated button LED code
    digitalWrite(UNREG_LED_PIN, unregLEDState);
}

// ==================================
// === INTERRUPT SERVICE ROUTINES ===
// ==================================

void hwregBtnISR() {
    hwregBtnPresses++; // Increment button presses
    Serial.println("Hardware-regulated button pressed!");
    Serial.print("It has been pressed "); Serial.print(hwregBtnPresses); Serial.println(" times!");
    Serial.println();
    hwregLEDState = !hwregLEDState; // Change LED state
}

void swregBtnISR() {
    static unsigned long _lastInterruptTime = 0;
    unsigned long _curInterruptTime = millis();
    if (_curInterruptTime - _lastInterruptTime > 100) { // If interrupts come faster than 100ms, assume it's a bounce and ignore
        swregBtnPresses++; // Increment button presses
        Serial.println("Software-regulated button pressed!");
        Serial.print("It has been pressed "); Serial.print(swregBtnPresses); Serial.println(" times!");
        Serial.println();
        swregLEDState = !swregLEDState; // Change LED state
    }
    _lastInterruptTime = _curInterruptTime;
}

void unregBtnISR() {
    unregBtnPresses++; // Increment button presses
    Serial.println("Un-regulated button pressed!");
    Serial.print("It has been pressed "); Serial.print(unregBtnPresses); Serial.println(" times!");
    Serial.println();
    unregLEDState = !unregLEDState; // Change LED state
}