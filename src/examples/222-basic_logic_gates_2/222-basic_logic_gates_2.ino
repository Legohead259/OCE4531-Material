/**
 * @file basic_logic_gates_2.ino
 * @author Braidan Duffy (bduffy2018@my.fit.edu) 
 * @brief Example code for the Basic Logic Gates 2 example
 * @version 1.0
 * @date 2022-08-13
 * Last Modified 2022-08-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>

#define A 2
#define B 3
#define C 4
#define D 5

uint8_t inputPins[4] = {A, B, C, D};

void setup() {    
    for (uint8_t i=0; i<4; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);
    }

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    if (!digitalRead(A) & !digitalRead(B) | !(!digitalRead(C) | !digitalRead(D))) // Check logic expression
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED
    else 
        digitalWrite(LED_BUILTIN, LOW); // Turn off LED
    delay(125);
}