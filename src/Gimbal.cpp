//
// Created by Misha on 4/17/2022.
//

#include "Gimbal.h"
#include "Arduino.h"

#define SERVO_MAX_US    1900
#define SERVO_MIN_US    1100
#define SERVO_RANGE_US  (SERVO_MAX_US - SERVO_MIN_US)
#define SERVO_MAX_RAD   (PI/2.0f)
#define SERVO_MIN_RAD   (-PI/2.0f)
#define SERVO_RANGE_RAD (SERVO_MAX_RAD - SERVO_MIN_RAD)
#define GEAR_RATIO      1.5f

Gimbal::Gimbal(int servo_pin) {
    this->servo_pin = servo_pin;
}

void Gimbal::begin() {
    servo.attach(servo_pin);
}

void Gimbal::setAngle(float angle) {
    int us = (int) ((angle * GEAR_RATIO - SERVO_MIN_RAD) / SERVO_RANGE_RAD * SERVO_RANGE_US + SERVO_MIN_US);
    servo.writeMicroseconds(us);
}


