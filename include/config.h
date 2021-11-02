//
// Created by Admin on 2021. 11. 02..
//

#ifndef ROBOTEXTRUDER_CONFIG_H
#define ROBOTEXTRUDER_CONFIG_H

#include <Arduino.h>

#define MAX_EXTRUDE_SPEED   100

#define EXTRUDE_PIN         43  //PD0 INT0
#define RETRACT_PIN         45  //PD1 INT2
#define SPEED_SCALE_PIN_0   0
#define SPEED_SCALE_PIN_1   0
#define SPEED_SCALE_PIN_2   0
#define SPEED_SCALE_PIN_3   0

// E0
#define STEPPER_DIR_PIN     28
#define STEPPER_EN_PIN      24
#define STEPPER_PULSE_PIN   26

#endif //ROBOTEXTRUDER_CONFIG_H
