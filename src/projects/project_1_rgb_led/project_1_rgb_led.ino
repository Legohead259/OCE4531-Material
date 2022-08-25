/**
 * OCE4531 Project 1: RGB LED Cycler Solution
 * @brief This is the solution for OCE4531's first project, the RGB LED cycler.
 * This should not be considered the absolute correct answer that all of the students must have,
 * but it is the ball park they should be within. 
 * 
 * @author Braidan Duffy
 * 
 * @date May 24, 2022
 * 
 * @version v1.0
 */

#define BUTTON_PIN 2
#define LED_RED_PIN 11
#define LED_GREEN_PIN 10
#define LED_BLUE_PIN 9
#define MAX_NUM_FUNCS 5
#define PI 3.1415

uint8_t ledMode = 0; // Initialize LED mode state

void setup() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial connection - DEBUG

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
}

void loop() {
    bool isButtonPressed = !digitalRead(BUTTON_PIN);
    if (isButtonPressed) {
        if (ledMode == MAX_NUM_FUNCS-1)
            ledMode = 0;
        else
            ledMode++;
        Serial.println(ledMode); //DEBUG
    }

    switch(ledMode) {
        case 0:
            breathe(true, false, false); // Breathe RED
            break;
        case 1:
            breatheSine(true, false, true); // Breath RED and BLUE
            break;
        case 2:
            rainbow();
            break;
        case 3:
            staticColor(false, false, true, 128); // Static BLUE
            break;
        default:
            delay(250); // Do nothing
            break;
    }

    // Clear LED Colors between cycles
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
}

// ==========================
// === LED MODE FUNCTIONS ===
// ==========================

void breathe(bool r, bool g, bool b) {
    for (uint8_t i=0; i<255; i++) { // Increase brightness
        if (r) analogWrite(LED_RED_PIN, i);
        if (g) analogWrite(LED_GREEN_PIN, i);
        if (b) analogWrite(LED_BLUE_PIN, i);

        delay(5);
    }

    for (uint8_t j=255; j>0; j--) {
        if (r) analogWrite(LED_RED_PIN, j);
        if (g) analogWrite(LED_GREEN_PIN, j);
        if (b) analogWrite(LED_BLUE_PIN, j);

        delay(5);
    }
}

void breatheSine(bool r, bool g, bool b) {
    for (int i=0; i<180; i++) {
        float iRad = i * PI / 180;
        float val = sin(iRad) * 255;

        if (r) analogWrite(LED_RED_PIN, val);
        if (g) analogWrite(LED_GREEN_PIN, val);
        if (b) analogWrite(LED_BLUE_PIN, val);

        delay(5);
    }
}

void rainbow() {
    for (int i=0; i<360; i++) {
        float iRad = i * PI / 180;
        float rVal = (sin(iRad)+1) * 128;
        float gVal = (sin(iRad + 2*PI/3) + 1) * 128;
        float bVal = (sin(iRad + 4*PI/3)+1) * 128;

        analogWrite(LED_RED_PIN, rVal);
        analogWrite(LED_GREEN_PIN, gVal);
        analogWrite(LED_BLUE_PIN, bVal);

        delay(5);
    }
}

void staticColor(bool r, bool g, bool b, uint8_t i) {
    if (r) analogWrite(LED_RED_PIN, i);
    if (g) analogWrite(LED_GREEN_PIN, i);
    if (b) analogWrite(LED_BLUE_PIN, i);

    delay(250);
}