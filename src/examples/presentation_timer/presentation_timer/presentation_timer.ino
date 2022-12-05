// Define the pin numbers for the LEDs
const int GREEN_LED_PIN = 12;
const int YELLOW_LED_PIN = 11;
const int RED_LED_PIN = 10;

void setup() {
  // Set the pin modes for the LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  // Turn on the green LED
  digitalWrite(GREEN_LED_PIN, HIGH);

  // Wait for 8 minutes
  delay(8 * 60 * 1000);

  // Turn off the green LED and turn on the yellow LED
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, HIGH);

  // Wait for 2 minutes
  delay(2 * 60 * 1000);

  // Turn off the yellow LED and turn on the red LED
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
}

void loop() {
  // The loop function is not used in this example
}
