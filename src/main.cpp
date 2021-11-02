#include <Arduino.h>
#include <AccelStepper.h>

#include "config.h"

AccelStepper motor = AccelStepper(AccelStepper::DRIVER, STEPPER_PULSE_PIN, STEPPER_EN_PIN);
boolean motorIsRunning = false;
boolean motorIsStopping = false;

void pinSetup() {
    pinMode(STEPPER_PULSE_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);

    pinMode(EXTRUDE_PIN, INPUT);
    pinMode(RETRACT_PIN, INPUT);

    pinMode(SPEED_SCALE_PIN_0, INPUT);
    pinMode(SPEED_SCALE_PIN_1, INPUT);
    pinMode(SPEED_SCALE_PIN_2, INPUT);
    pinMode(SPEED_SCALE_PIN_3, INPUT);
}

void ISRSetup() {
    //control register
    EICRA = 0;
    EICRA |= (1 << ISC00) | (1 << ISC10); // falling edge detection

    //mask register
    EIMSK = 0;
    EIMSK |= (1 << INT0) | (1 << INT2);
}

void setup() {
    pinSetup();
    ISRSetup();

    motor.setEnablePin(STEPPER_EN_PIN);
    motor.setSpeed(MAX_EXTRUDE_SPEED);
}

void loop() {
// write your code here
    if (motorIsRunning) {
        motor.runSpeed();
    }
    if (motorIsStopping) {
        motor.stop();

        motorIsRunning = false;
        motorIsStopping = false;
    }
}



/* INTERRUPTS */

ISR (INT0_vect) {
    printf("Interrupt detected");
    if (digitalRead(EXTRUDE_PIN) == HIGH) { // LOW -> HIGH
        motorIsRunning = true;
    }
    else { // HIGH -> LOW
        motorIsStopping = true;
    }
}
