#include <PinChangeInterrupt.h>  // Library for handling external interrupts
#include <TimerOne.h>  // Library for handling timer-based interrupts

// Define pin numbers
#define INTERRUPT_SENSOR_1_PIN 4  // First interrupt sensor (e.g., motion sensor)
#define INTERRUPT_SENSOR_2_PIN 5  // Second interrupt sensor
#define TRIG_PIN 7  // Ultrasonic sensor trigger pin
#define ECHO_PIN 6  // Ultrasonic sensor echo pin
#define TIMER_LED_PIN 8  // LED that blinks using a timer interrupt

// Variables to track if an interrupt has occurred
volatile bool interrupt1Triggered = false;
volatile bool interrupt2Triggered = false;

// Function to handle interrupt from Sensor 1
void interrupt1ISR() {
    interrupt1Triggered = true;
}

// Function to handle interrupt from Sensor 2
void interrupt2ISR() {
    interrupt2Triggered = true;
}

// Function that runs every 2 seconds (Timer Interrupt)
void timerISR() {
    digitalWrite(TIMER_LED_PIN, HIGH); // Turn LED on
    delay(1000);  // Wait for 100 milliseconds
    digitalWrite(TIMER_LED_PIN, LOW); // Turn LED off
}

// Function to measure distance using an Ultrasonic Sensor
long getDistance() {
    digitalWrite(TRIG_PIN, LOW); // Ensure trigger is LOW before starting
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); // Send 10-microsecond pulse
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH); // Measure echo return time
    long distance = duration * 0.034 / 2;  // Convert time to distance in cm
    return distance;
}

void setup() {
    Serial.begin(9600);  // Start serial communication at 9600 baud rate

    // Set pin modes
    pinMode(INTERRUPT_SENSOR_1_PIN, INPUT);  // Sensor 1 as input
    pinMode(INTERRUPT_SENSOR_2_PIN, INPUT);  // Sensor 2 as input
    pinMode(TRIG_PIN, OUTPUT);  // Ultrasonic sensor trigger as output
    pinMode(ECHO_PIN, INPUT);   // Ultrasonic sensor echo as input
    pinMode(TIMER_LED_PIN, OUTPUT);  // LED as output

    // Attach interrupt functions to sensors (triggered when signal goes LOW)
    attachPCINT(digitalPinToPCINT(INTERRUPT_SENSOR_1_PIN), interrupt1ISR, FALLING);
    attachPCINT(digitalPinToPCINT(INTERRUPT_SENSOR_2_PIN), interrupt2ISR, FALLING);

    // Initialize timer interrupt to run every 2 seconds
    Timer1.initialize(2000000);  // 2,000,000 microseconds = 2 seconds
    Timer1.attachInterrupt(timerISR);  // Attach the timer function
}

void loop() {
    // Check if Sensor 1 was triggered
    if (interrupt1Triggered) {
        Serial.println("Interrupt Sensor 1 Triggered!");
        interrupt1Triggered = false; // Reset the flag
    }

    // Check if Sensor 2 was triggered
    if (interrupt2Triggered) {
        Serial.println("Interrupt Sensor 2 Triggered!");
        interrupt2Triggered = false; // Reset the flag
    }

    // Measure and display distance from ultrasonic sensor
    long distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    delay(500); // Wait 500 milliseconds before next measurement
}
