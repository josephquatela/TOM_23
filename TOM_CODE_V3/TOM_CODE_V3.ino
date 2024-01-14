/*
Code for the 2023 TOM competition.
@Blake Wolf, Joseph Quatela, Grey Aycock
Vanderbilt University
*/

#include "avr/interrupt.h"
#include "avr/io.h"
#include "avr/sleep.h"

/* Define GPIO Names */
// LEDS and Button
#define LED0 9
#define LED1 10
#define LED2 0
#define LED3 1
#define BUTTON1 2
#define BUTTON2 3

// Motor Control
#define FORWARD 7
#define BACKWARD 6

// Limit Switches
#define LS1 4
#define LS2 5

// Battery Voltage
#define BATTERY_PIN 8


/* Constants */
// Sleep Timeout Value
#define TIMEOUT_VALUE 420000 // about 5 minutes

// Device States
#define SLEEPING 0
#define RETRACTED 1
#define EXTENDED 2
#define EXTENDING 3
#define RETRACTING 4
#define WAKING 5

// Max Battery Voltage
#define MAX_BAT 2.1
#define MIN_BAT 1.125

/* Setup Data Vars */
int buttonSignal;
int deviceState;
int extending;
int retracting;
long curTime;
float batteryLevel;

/* Function Declarations */
void pulseLED(int LED, int time_ms);
void testLEDs(int time_ms);
void testMotors(int time_ms);
void extend();
void enableInterrupt();
void goToSleep();
float getBatteryLevel();
void flashBatteryLevel();

// pulseLED - Pulses the given LED for "time_ms" amount of time (in ms)
void pulseLED(int LED, int time_ms) {
  digitalWrite(LED, 1);
  delay(time_ms);
  digitalWrite(LED, 0);
}

// testLEDs - Function to pulse each LED and make sure they are working properly
void testLEDs(int time_ms) {
  pulseLED(LED0, time_ms);
  pulseLED(LED1, time_ms);
  pulseLED(LED2, time_ms);
  pulseLED(LED3, time_ms);
}

// testMotors - Function to drive the motor forward and then backward for a short time (sanity check)
void testMotors(int time_ms) {
  digitalWrite(FORWARD, 1);
  delay(time_ms);
  digitalWrite(FORWARD, 0);
  delay(time_ms);
  digitalWrite(BACKWARD, 1);
  delay(time_ms);
  digitalWrite(BACKWARD, 0);
}

// extend - Extends the shaft to the limit switch with visual indication
void extend() {
  digitalWrite(FORWARD, 1);
  extending = 1;
  while (extending) {
    // wait until limit switch is hit
    extending = digitalRead(LS1);
    // do a Dancy Dance while waiting
    testLEDs(200);
  }
  digitalWrite(FORWARD, 0);
}

// retract - Retracts the shaft to the limit switch with visual indication
void retract() {
  digitalWrite(BACKWARD, 1);
  retracting = 1;
  while (retracting) {
    // wait until limit switch is hit
    retracting = digitalRead(LS2);
    // do a Dancy Dance while waiting
    pulseLED(LED3, 200);
    pulseLED(LED2, 200);
    pulseLED(LED1, 200);
    pulseLED(LED0, 200);
  }
  digitalWrite(BACKWARD, 0);
}

// enableInterrupt - Enable interrupts for PA6
void enableGPIOInterrupt() {
  // Enable rising edge triggered interrupt
  PORTA.PIN6CTRL = (PORTA.PIN6CTRL & ~PORT_ISC_gm) | (PORT_ISC_RISING_gc);
  sei();
}

// goToSleep - Puts device in sleep mode and enables interrupts for wakeup
void goToSleep() {
  // Enable interrupt for button1 (PA6, Pin 4, Arduino pin 2)
  enableGPIOInterrupt();
 
  // Enter sleep mode
  deviceState = SLEEPING;
  SLPCTRL.CTRLA = SLEEP_MODE_IDLE | SLEEP_ENABLED_gc;
  sleep_cpu();
}

// getBatteryLevel
float getBatteryLevel() {
  float batteryReading;
  float curVoltage;
  curVoltage = MAX_BAT - analogRead(BATTERY_PIN);
  batteryReading = curVoltage / (MAX_BAT - MIN_BAT);
  return batteryReading;
}

// flashBatteryLevel
void flashBatteryLevel() {
  if (batteryLevel <= 0.25 ) {
    digitalWrite(LED0, 1);
    delay(1000);
    digitalWrite(LED0, 0);
  } else if (batteryLevel <= 0.5) {
    digitalWrite(LED0, 1);
    digitalWrite(LED1, 1);
    delay(1000);
    digitalWrite(LED0, 0);
    digitalWrite(LED1, 0);
  } else if (batteryLevel <= 0.75) {
    digitalWrite(LED0, 1);
    digitalWrite(LED1, 1);
    digitalWrite(LED2, 1);
    delay(1000);
    digitalWrite(LED0, 0);
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
  } else if (batteryLevel <= 1) {
    digitalWrite(LED0, 1);
    digitalWrite(LED1, 1);
    digitalWrite(LED2, 1);
    digitalWrite(LED3, 1);
    delay(1000);
    digitalWrite(LED0, 0);
    digitalWrite(LED1, 0);
    digitalWrite(LED2, 0);
    digitalWrite(LED3, 0);
  } else {
    digitalWrite(LED0, 1);
    digitalWrite(LED2, 1);
    delay(1000);
    digitalWrite(LED0, 0);
    digitalWrite(LED2, 0);
    digitalWrite(LED1, 1);
    digitalWrite(LED3, 1);
    delay(1000);
    digitalWrite(LED1, 0);
    digitalWrite(LED3, 0);
  }
}

// Wakeup ISR
ISR(PORTA_PORT_vect, ISR_BLOCK) {
  // Update deviceState and return. Disable interrupts on next instruction.
  deviceState = RETRACTED;
  
  // Disable GPIO interrupt
  PORTA.PIN6CTRL = 0x00;
  // Clear Interrupt Flag by writing 1's
  PORTA.INTFLAGS = 0xFF;

  // Display Battery Level
  batteryLevel = getBatteryLevel();
  flashBatteryLevel();

  // Disable sleep
  SLPCTRL.CTRLA = 0x00;

  reti();
}

// setup - Runs once at startup
void setup() {
  // Setup GPIO pins
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(LS1, INPUT);
  pinMode(LS2, INPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);

  // Test basic functions
  testLEDs(1000);
  delay(1000);
  testMotors(500);

  // Set initial state to retracted (homed and not sleeping)
  deviceState = RETRACTED;
}

// loop - Runs repeatedly
void loop() {
  // "Retracted" State Control
  if (deviceState == RETRACTED) {
    // Listen for extension signal
    buttonSignal = digitalRead(BUTTON1);
    if (buttonSignal) {
      // Reset timeout count
      curTime = 0;
      // Extend
      deviceState = EXTENDING;
      extend();
      deviceState = EXTENDED;
    }
  }

  // "Extended" State Control
  if (deviceState == EXTENDED) {
    // Listen for retraction signal
    buttonSignal = digitalRead(BUTTON1);
    if (buttonSignal) {
      // Reset timeout count
      curTime = 0;
      // Retract
      deviceState = RETRACTING;
      retract();
      deviceState = RETRACTED;
    }
  }

  // Timeout (sleep) Control
  curTime = millis();
  // Check if timeout time has been reached
  if (curTime > TIMEOUT_VALUE) {
    curTime = 0;
    // Retract shaft if extended before sleeping
    if (deviceState == EXTENDED) {
      deviceState = RETRACTING;
      retract();
      deviceState = RETRACTED;
    }

    goToSleep();
  }
}


