/**
 * @file example_2_1_blink.ino
 * @author Braidan Duffy (bduffy2018@my.fit.edu)
 * @brief Example code used to demonstrate binary
 * @version 1.0
 * @date 2022-08-13
 * Date Modified: 2022-08-13
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}

