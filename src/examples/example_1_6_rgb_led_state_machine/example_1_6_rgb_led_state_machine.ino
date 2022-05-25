/**
 * OCE4531 Example 1.5: RGB LED State Machine
 * @brief This is the source code for Example 1.5 found in the OCE4531 lecture notes. 
 * Hookup the Arduino and breadboard as indicated in the example write-up in the lecture notes
 * 
 * @author Braidan Duffy
 * 
 * @date May 25, 2022
 * 
 * @version v1.0
 */

// Initialize constant variables
#define BUTTON_PIN      2
#define LED_RED_PIN     12
#define LED_GREEN_PIN   11
#define LED_BLUE_PIN    10

#define MAX_NUM_FUNCS   3

uint8_t ledMode = 0; // Initialize LED mode state

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP); // INPUT_PULLUP specifies that an internal resistor pulls up the pin, when we press the button, this pin will go LOW, creating a clear edge we can use for press detection
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
}

void loop() {
    bool isButtonPressed = !digitalRead(BUTTON_PIN);
    if (isButtonPressed) {                  // On button press, move the state machine forward
        if (ledMode == MAX_NUM_FUNCS-1)     // If the counter is in the final state, reset it
            ledMode = 0;
        else                                // Otherwise increment the state counter
            ledMode++;
    }

    switch(ledMode) {
        case 0:
            analogWrite(LED_RED_PIN, 255); // Writes a full DC PWM wave to glow bright RED
            break;
        case 1:
            analogWrite(LED_GREEN_PIN, 128); // Writes a half DC PWM wave to glow dim GREEN
            break;
        case 2:
            analogWrite(LED_BLUE_PIN, 64); // Writes a low DC PWM wave to glow dimmer BLUE
            break;
    }
    delay(250);

    // Clear LED Colors between cycles
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
}