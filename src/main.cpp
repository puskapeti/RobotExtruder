#include <Arduino.h>
#include <AccelStepper.h>

#include "config.h"

void pinSetup();
void ISRSetup();

void stepForward();
void stepBackward();
void enableMotor();
void disableMotor();

void extruderInterruptCallback();
void retractInterruptCallback();

enum MotorState {
    extruding, stopping, retracting, not_running
};

AccelStepper motor = AccelStepper(stepForward, stepBackward);
MotorState motorState = not_running;

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    pinSetup();
    ISRSetup();

    motor.setMaxSpeed(MAX_EXTRUDE_SPEED);
    motor.setAcceleration(MOTOR_ACCELERATION);
    enableMotor();
    motor.move(DEG2STEPS(360));
}

MotorState prevState = MotorState::not_running;
void loop() {
    long stepsToStop = (long)((MAX_EXTRUDE_SPEED * MAX_EXTRUDE_SPEED) / (2.0 * MOTOR_ACCELERATION));
    switch (motorState) {
        case extruding:
            motor.run();
            if (motor.distanceToGo() < DEG2STEPS(stepsToStop) * 2) {
                motor.move(DEG2STEPS(360));
            }
            prevState = extruding;
            break;

        case stopping:
            if (prevState == extruding) {
                motor.stop(); //calculates new target position
                prevState = stopping;
            }
            motor.run();
            if (motor.distanceToGo() == 0) {
                motorState = not_running;
            }
            break;

        case retracting:
            //TODO implement this
            break;

        case not_running:
            //TODO implement this
            break;
    }
#ifdef DEBUG
    if(prevState != motorState) {
        DPRINT("prev. state: ");
        DPRINT(prevState);
        DPRINT("curr. state: ");
        DPRINT(motorState);
    }
#endif
}

void pinSetup() {
    pinMode(STEPPER_PULSE_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);

    pinMode(EXTRUDE_PIN, INPUT_PULLUP);
    pinMode(RETRACT_PIN, INPUT_PULLUP);

    pinMode(SPEED_SCALE_PIN_0, INPUT);
    pinMode(SPEED_SCALE_PIN_1, INPUT);
    pinMode(SPEED_SCALE_PIN_2, INPUT);
    pinMode(SPEED_SCALE_PIN_3, INPUT);
}

void ISRSetup() {
    attachInterrupt(digitalPinToInterrupt(EXTRUDE_PIN), extruderInterruptCallback, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RETRACT_PIN), retractInterruptCallback, RISING);
}

void stepForward() {
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    digitalWrite(STEPPER_PULSE_PIN, HIGH);
    digitalWrite(STEPPER_PULSE_PIN, LOW);
}

void stepBackward() {
    digitalWrite(STEPPER_DIR_PIN, LOW);
    digitalWrite(STEPPER_PULSE_PIN, HIGH);
    digitalWrite(STEPPER_PULSE_PIN, LOW);
}

void enableMotor() {
    digitalWrite(STEPPER_EN_PIN, LOW);
}

void disableMotor() {
    digitalWrite(STEPPER_PULSE_PIN, HIGH);
}

#ifdef SERIAL_CONTROL
void serialEvent() {
    String msg = Serial.readString();
    if (msg == "extrude\n") {
        if (!motorIsRunning) {
            motorIsRunning = true;
            Serial.println("Motor running");
        }
        else {
            motorIsStopping = true;
            Serial.println("Motor stopping");
        }
    }
}
#endif

unsigned long eISRLastCalled = 0;

/* INTERRUPTS */
void extruderInterruptCallback() {
#ifdef IO_DEBOUNCE
    unsigned long eISRCalled = millis();
    if (eISRCalled - eISRLastCalled < DEBOUNCE_TIME) {
        //bouncing
        return;
    }
    else {
        //stopped bouncing
        eISRLastCalled = eISRCalled;
    }
#endif
    DPRINTLN("Extruder callback");
    if (digitalRead(EXTRUDE_PIN) == HIGH) { // rising edge
/*        byte speedScale = 0b00000001;
        if (digitalRead(SPEED_SCALE_PIN_0) == HIGH) {
            speedScale |= (1 << 0);
        }
        if (digitalRead(SPEED_SCALE_PIN_1) == HIGH) {
            speedScale |= (1 << 1);
        }
        if (digitalRead(SPEED_SCALE_PIN_2) == HIGH) {
            speedScale |= (1 << 2);
        }
        if (digitalRead(SPEED_SCALE_PIN_3) == HIGH) {
            speedScale |= (1 << 3);
        }
        int speedScaleInt = (int) speedScale;
        float speed = MAX_EXTRUDE_SPEED / (float) speedScaleInt;
        motor.setSpeed(speed);
        DPRINT(speed);
        DPRINT(speedScaleInt);*/

        motorState = extruding;
    }
    else { // falling edge
        motorState = stopping;
    }
    DPRINTLN(motorState);
}

unsigned long rISRLastCalled = 0;

void retractInterruptCallback() {
#ifdef IO_DEBOUNCE
    unsigned long rISRCalled = millis();
    if (rISRCalled - rISRLastCalled < DEBOUNCE_TIME) {
        //bouncing
        return;
    }
    else {
        //stopped bouncing
        rISRLastCalled = rISRCalled;
    }
#endif
    DPRINTLN("Retract callback");
    motorState = retracting;
}
