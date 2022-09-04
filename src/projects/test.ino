#define GREEN_LED_PIN 5
#define RED_LED_PIN 6

void setup() {
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
}

void loop() {
    blink(GREEN_LED_PIN, 1000);

    // ...
    float i2 = square(55); // Returns 55^2 = 3025 = i2

    // Some other use case where I want to blink a different pin at a different speed
    while (true) {
        blink(RED_LED_PIN, 500);
    }
}

void blink(uint8_t pin, int delay) {
    digitalWrite(pin, HIGH);
    delay(delay);
    digitalWrite(pin, LOW);
    delay(delay);
}

float square(int i) {
    return i*i;
}

float power(int i, int n) {
    float _pow = 0;
    for (int k=0; k<i; k++) {
        _pow *= i;
    }
    return _pow;
}

void someFunction() {
    char _str[4] = {'W', 'O', 'R', 'D'};
    for (int i=0; i<4; i++) {
        Serial.print(_str[i]);
    }
    Serial.println();
    // Expectation: WORD
    //              WORD
    //              WORD [...]
}
