//
// Created by Misha on 4/17/2022.
//

#include <Arduino.h>
#include <Adafruit_MPU6050.h>

#include "Gimbal.h"

//#define VDEBUG

#define STATE_LOADING   0
#define STATE_ARMED     1
#define STATE_ACTIVE    2
#define STATE_DRINK     3

#define BUTTON_PIN      2

Adafruit_MPU6050 imu;
Gimbal gimbal_x(5), gimbal_y(6);

uint8_t machine_state = STATE_LOADING;

void setup() {
#ifdef VDEBUG
    Serial.begin(115200);
#endif

    // Initialize MPU-6050 IMU
    if (!imu.begin()) {
#ifdef VDEBUG
        Serial.println("Couldn't find MPU-6050!");
#endif
        while (true);
    }

    // Set IMU operating characteristics
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // Initialize gimbals
    gimbal_x.begin();
    gimbal_y.begin();

    // Initialize inputs
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    machine_state = STATE_ARMED;
}

void loop() {
    static unsigned long last_cycle = millis();
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    float gim_x_eff_g_angle = atan2f(accel.acceleration.y, -accel.acceleration.x);
    float gim_y_eff_g_angle = atan2f(accel.acceleration.z, -accel.acceleration.x);

    switch (machine_state) {
        case STATE_ARMED:
            gimbal_x.setAngle(0);
            gimbal_y.setAngle(0);

            if (digitalRead(BUTTON_PIN) == LOW)
                machine_state = STATE_ACTIVE;
            break;

        case STATE_ACTIVE:
            gimbal_x.setAngle(gim_x_eff_g_angle);
            gimbal_y.setAngle(gim_y_eff_g_angle);

            if (digitalRead(BUTTON_PIN) == LOW)
                machine_state = STATE_DRINK;
            break;

        case STATE_DRINK:
            gimbal_x.setAngle(gim_x_eff_g_angle + ((float) PI / 6.0f));
            gimbal_y.setAngle(gim_y_eff_g_angle);

            if (digitalRead(BUTTON_PIN) == HIGH)
                machine_state = STATE_ACTIVE;
            break;

        default:
            break;
    }

    while (millis() - last_cycle < 20);
    last_cycle = millis();

#ifdef VDEBUG
    Serial.print(gim_x_eff_g_angle);
    Serial.print('\t');
    Serial.print(gim_y_eff_g_angle);
    Serial.println();
#endif
}