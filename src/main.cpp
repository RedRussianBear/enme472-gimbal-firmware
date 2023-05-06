//
// Created by Misha on 4/17/2022.
//

#include <Arduino.h>
#include <EEPROM.h>
#include <Adafruit_MPU6050.h>

#include "Gimbal.h"

//#define VDEBUG

#define STATE_LOADING   0
#define STATE_ARMED     1
#define STATE_ACTIVE    2
#define STATE_DRINK     3

#define BUTTON_PIN      2

#define MIN_CYCLE_TIME  20

Adafruit_MPU6050 imu;
Gimbal gimbal_x(9), gimbal_y(10);

uint8_t machine_state = STATE_LOADING;
float gim_x_eff_g_angle;
float gim_y_eff_g_angle;

struct __attribute__((__packed__)) StoredParameters {
    float ax_off, ay_off, az_off, gx_off, gy_off, gz_off;
    float gim_x_off, gim_y_off;
} parameters;
int param_address = 0;

void setup() {
#ifdef VDEBUG
    Serial.begin(115200);
#endif
    EEPROM.get(param_address, parameters);

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
    imu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // Initialize gimbals
    gimbal_x.begin();
    gimbal_y.begin();

    // Initialize inputs
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    machine_state = STATE_ARMED;
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    gim_x_eff_g_angle = atan2f(accel.acceleration.y, -accel.acceleration.x);
    gim_y_eff_g_angle = atan2f(accel.acceleration.z, -accel.acceleration.x);
}

void loop() {
    static unsigned long last_cycle = millis();
    float dt = (float) (millis() - last_cycle) / 1000.0f;
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    float ax = accel.acceleration.x + parameters.ax_off;
    float ay = accel.acceleration.y + parameters.ay_off;
    float az = accel.acceleration.z + parameters.az_off;
    float gx = gyro.gyro.x + parameters.gx_off;
    float gy = gyro.gyro.y + parameters.gy_off;
    float gz = gyro.gyro.z + parameters.gz_off;

    float gim_x_mag = ax*ax + ay*ay;
    float gim_y_mag = az*az + ax*ax;
    float gim_x_ang_est = (gim_x_mag > 8*8 && gim_x_mag < 11*11) ? atan2f(ay, -ax) : gim_x_eff_g_angle;
    float gim_y_ang_est = (gim_y_mag > 8*8 && gim_y_mag < 11*11) ? atan2f(az, -ax) : gim_y_eff_g_angle;

    gim_x_eff_g_angle = 0.25f * gim_x_ang_est
                        + 0.75f * (gim_x_eff_g_angle + -gz * dt);
    gim_y_eff_g_angle = 0.25f * gim_y_ang_est
                        + 0.75f * (gim_y_eff_g_angle + -gy * dt);

    switch (machine_state) {
        case STATE_ARMED:
            gimbal_x.setAngle(parameters.gim_x_off);
            gimbal_y.setAngle(-parameters.gim_y_off);

            if (digitalRead(BUTTON_PIN) == LOW)
                machine_state = STATE_ACTIVE;
            break;

        case STATE_ACTIVE:
            gimbal_x.setAngle(gim_x_eff_g_angle + parameters.gim_x_off);
            gimbal_y.setAngle(-(gim_y_eff_g_angle + parameters.gim_y_off));

            if (digitalRead(BUTTON_PIN) == LOW)
                machine_state = STATE_DRINK;
            break;

        case STATE_DRINK:
            gimbal_x.setAngle(parameters.gim_x_off);
            gimbal_y.setAngle(-(gim_y_eff_g_angle + parameters.gim_y_off));

            if (digitalRead(BUTTON_PIN) == HIGH)
                machine_state = STATE_ACTIVE;
            break;

        default:
            break;
    }

#ifdef VDEBUG
    Serial.print(ax);
    Serial.print('\t');
    Serial.print(ay);
    Serial.print('\t');
    Serial.print(az);
    Serial.print('\t');
    Serial.print(gx);
    Serial.print('\t');
    Serial.print(gy);
    Serial.print('\t');
    Serial.print(gz);
    Serial.print('\t');
    Serial.print(gim_x_eff_g_angle);
    Serial.print('\t');
    Serial.print(gim_y_eff_g_angle);
    Serial.println();

    // Instrument and trim tuning
    if (Serial.available()) {
        int b1 = Serial.read();
        int b2 = Serial.read();

        switch (b1) {
            case 'a': // Accel
                switch (b2) {
                    case 'x':
                        parameters.ax_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    case 'y':
                        parameters.ay_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    case 'z':
                        parameters.az_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    default:
                        break;
                }
                break;
            case 'g': // Gyro
                switch (b2) {
                    case 'x':
                        parameters.gx_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    case 'y':
                        parameters.gy_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    case 'z':
                        parameters.gz_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    default:
                        break;
                }
                break;
            case 't': // Trim
                switch (b2) {
                    case 'x':
                        parameters.gim_x_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    case 'y':
                        parameters.gim_y_off = Serial.parseFloat();
                        EEPROM.put(param_address, parameters);
                        break;
                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
#endif

    while (millis() - last_cycle < MIN_CYCLE_TIME);
    last_cycle = millis();
}