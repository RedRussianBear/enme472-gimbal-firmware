//
// Created by Misha on 4/17/2022.
//

#ifndef ENME472_GIMBAL_FIRMWARE_GIMBAL_H
#define ENME472_GIMBAL_FIRMWARE_GIMBAL_H


#include <Servo.h>

class Gimbal {
public:
    Gimbal(int servo_pin);
    void begin();
    void setAngle(float angle);

private:
    Servo servo;
    int servo_pin;

};


#endif //ENME472_GIMBAL_FIRMWARE_GIMBAL_H
