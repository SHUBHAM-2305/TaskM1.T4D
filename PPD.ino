#include <PinChangeInterrupt.h>
#include <TimerOne.h>  // Timer interrupt library

// Define pins
#define PIR_SENSOR_PIN 4
#define INTERRUPT_SENSOR_PIN 5
#define TRIG_PIN 7
#define ECHO_PIN 6
#define TIMER_LED_PIN 8  // LED controlled by Timer Interrupt

volatile bool pirTriggered = false;
volatile bool interruptTriggered = false;

// PIR Sensor Interrupt Service Routine
void pirISR() {
    pirTriggered = true;
}

// External Interrupt Sensor ISR
void interruptISR() {
    interruptTriggered = true;
}

// Timer Interrupt Service Routine (Fires every 2 seconds)
void timerISR() {
    digitalWrite(TIMER_LED_PIN, HIGH); // Turn on LED
}

// Function to measure distance using Ultrasonic Sensor
long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

void setup() {
    Serial.begin(9600);

    // Set pin modes
    pinMode(PIR_SENSOR_PIN, INPUT);
    pinMode(INTERRUPT_SENSOR_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(TIMER_LED_PIN, OUTPUT); // LED output

    // Attach pin change interrupts
    attachPCINT(digitalPinToPCINT(PIR_SENSOR_PIN), pirISR, RISING);
    attachPCINT(digitalPinToPCINT(INTERRUPT_SENSOR_PIN), interruptISR, FALLING);

    // Initialize timer interrupt (fires every 2 seconds)
    Timer1.initialize(2000000);  // 2,000,000 microseconds = 2 seconds
    Timer1.attachInterrupt(timerISR);
}

void loop() {
    if (pirTriggered) {
        Serial.println("PIR Sensor Triggered!");
        pirTriggered = false;
    }

    if (interruptTriggered) {
        Serial.println("Interrupt Sensor Triggered!");
        interruptTriggered = false;
    }

    long distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    delay(500); // Stability delay
}
