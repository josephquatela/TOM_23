/*
Code for the 2023 TOM competition.
@Blake Wolf, Joseph Quatela, Grey Aycock
Vanderbilt University
*/

#include "avr/sleep.h"
#include "avr/interrupt.h"
#include "avr/io.h"

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

/* Constants */

// Device States
#define SLEEPING 0
#define RETRACTED 1
#define EXTENDED 2
#define EXTENDING 3
#define RETRACTING 4

// Timeout
#define TIMEOUT_VALUE 36

/* Setup Data Vars */
int buttonSignal;
int deviceState;
int sleepyTime;
int extending;
int retracting;

/* Function Declarations */
void pulseLED(int LED, int time_ms);
void testLEDs(int time_ms);
void testMotors(int time_ms);
void extend();
void enableInterrupt();
void goToSleep();
void startTimer();
void stopTimer();

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
    testLEDs(500);
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
    pulseLED(LED3, 500);
    pulseLED(LED2, 500);
    pulseLED(LED1, 500);
    pulseLED(LED0, 500);
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
}

// Wakeup ISR
ISR(PORTA_PORT_vect, ISR_BLOCK) {
  // Update deviceState and return. Disable interrupts on next instruction.
  deviceState = RETRACTED;
  
  // Disable interrupt on button press
  PORTA.PIN6CTRL = 0x00;
  // Clear Interrupt Flag by writing 1's
  PORTA.INTFLAGS = 0xFF;

  // Start Timer
  startTimer();
  
  reti();
}

// startTimer - Sets up and starts 16-bit timer1 to count up
void startTimer() {
  // Set top value to max
  TCA0.SINGLE.PER = 0xFFFF;
  // Set timer to Normal Mode
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
  // Set pre-scalar and enable timer to start counting
  TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV1024_gc) | 0x01;
  // Enable interrupts at TOP
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
}

void stopTimer() {
  // Set all timer registers back to zero
  TCA0.SINGLE.PER = 0x0000;
  TCA0.SINGLE.CTRLB = 0x00;
  TCA0.SINGLE.INTCTRL = 0x00;
  TCA0.SINGLE.CTRLA = 0x00;
}

// Time Count ISR
ISR(TCA0_OVF_vect, ISR_BLOCK) {
  ++sleepyTime;
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

  // Initialize data vars
  sleepyTime = 0;

  // Start timer for timeout
  startTimer();
}

// loop - Runs repeatedly
void loop() {
  // "Retracted" State Control
  if (deviceState == RETRACTED) {
    // Listen for extension signal
    buttonSignal = digitalRead(BUTTON1);
    if (buttonSignal) {
      // Reset timeout count
      sleepyTime = 0;
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
      sleepyTime = 0;
      // Retract
      deviceState = RETRACTING;
      retract();
      deviceState = RETRACTED;
    }
  }

  // Timeout (sleep) Control
  if (sleepyTime == TIMEOUT_VALUE) {
    // Reset timeout counter and stop timer
    stopTimer();
    sleepyTime = 0;
    // Retract shaft if extended before sleeping
    if (deviceState == EXTENDED) {
      deviceState = RETRACTING;
      retract();
      deviceState = RETRACTED;
    }

    goToSleep();
  }
}


