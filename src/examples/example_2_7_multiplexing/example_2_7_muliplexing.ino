/**
 * @file example_2_7_multiplexing.ino
 * @brief Example code for the multiplexing example for the OCE4531 lecture notes.
 * @author Braidan Duffy
 * @date 08/13/2022
 * Modified: 08/13/2022
 */

#include <Arduino.h>

#define X1 22 // Pin for row 1
#define X2 24 // Pin for row 2
#define X3 26 // Pin for row 3
#define X4 28 // Pin for row 4
#define Y1 23 // Pin for col 1
#define Y2 25 // Pin for col 2
#define Y3 27 // Pin for col 3
#define Y4 29 // Pin for col 4

uint8_t rowPins[4] = {X1, X2, X3, X4}; // Array for row pins
uint8_t colPins[4] = {Y1, Y2, Y3, Y4}; // Array for col pins

char symbols[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void setup() {
    Serial.begin(9600);

    for (uint8_t i=0; i<4; i++) { // Initialize pin modes
        pinMode(rowPins[i], INPUT_PULLUP);
        pinMode(colPins[i], OUTPUT);
    }
}

void loop() {
    for (uint8_t c=0; c<4; c++) { // Iterate over column pins
        pin_mode(columnPins[c], OUTPUT);\
        digitalWrite(colPins[c], LOW); // Set column to low
        for (uint8_t r=0; r<4; r++) { // Iterate over row pins
            if (!digitalRead(rowPins[r])) { // Check row pin
                Serial.print(symbols[r][c]); Serial.println(" Pressed!");
            }
        }
        digitalWrite(colPins[c], HIGH); // Reset column to high
        pin_mode(columnPins[c], INPUT); // Reset column to high impedence
    }
    delay(125);
}