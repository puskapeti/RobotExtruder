//
// Created by Admin on 2021. 11. 02..
//

#ifndef ROBOTEXTRUDER_CONFIG_H
#define ROBOTEXTRUDER_CONFIG_H

#include <Arduino.h>

#define DEBUG
//#define SERIAL_CONTROL
#define IO_DEBOUNCE

#define MAX_EXTRUDE_SPEED   1000.0
#define MOTOR_ACCELERATION  1000.0

#define EXTRUDE_PIN         2  //PE4 INT4
#define RETRACT_PIN         3  //PE3 INT5
#define SPEED_SCALE_PIN_0   16
#define SPEED_SCALE_PIN_1   17
#define SPEED_SCALE_PIN_2   23
#define SPEED_SCALE_PIN_3   25

// E0
/*#define STEPPER_DIR_PIN     28
#define STEPPER_EN_PIN      24
#define STEPPER_PULSE_PIN   26*/

// E1
#define STEPPER_DIR_PIN     34
#define STEPPER_EN_PIN      30
#define STEPPER_PULSE_PIN   36

#define STEPS_PER_REV   200.0
#define MICROSTEPS      16.0

#define DEG2STEPS(deg)  ((float) deg / 360.0 * STEPS_PER_REV * MICROSTEPS)

#ifdef DEBUG
#define DPRINT(msg) Serial.print(msg)
#define DPRINTLN(msg) Serial.println(msg)
#else
#define DPRINT(msg)
#define DPRINTLN(msg)
#endif

//debouncing
#ifdef IO_DEBOUNCE
#define DEBOUNCE_TIME 50 //ms
#endif

#endif //ROBOTEXTRUDER_CONFIG_H
