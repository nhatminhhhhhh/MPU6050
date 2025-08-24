#include <pid.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <mpu.h>

/// PID control variables
const float Kp = 4.0;   // Proportional gain
const float Ki = 0.0;   // Integral gain
const float Kd = 0.3;   // Derivative gain

//Variables for timing
long prevTime = 0;
float eprev = 0;
float eintegral = 0;

// Target angle for the PID controller
volatile float mpu_set = 0.0; // Target angle (setpoint)

#define MaxOutput (255.0)
#define MinOutput (0.0)
#define CONSTRAIN(x, low, high)  ((x)<(low)?(low):((x)>(high)?(high):(x)))


void pid_calculate() {
    long now = (uint32_t)esp_timer_get_time();
    float deltaT= ((float)(now - prevTime) )/ 1000000.0; // Convert microseconds to seconds
    prevTime = now;

    float error = mpu_set - mpu_read();
    float derivative = (error - eprev) / deltaT;
    eintegral += error * deltaT;

    float output = (Kp * error) + (Ki * eintegral) + (Kd * derivative);
    int pwr = (int)CONSTRAIN(fabs(output), MinOutput, MaxOutput);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pwr);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    printf("MPU State: %.2f, Setpoint: %.2f, Error: %.2f, Output: %d\n", mpu_read(), mpu_set, error, pwr);

    eprev = error;

    // Debugging output
    //printf("Output PID: %.2f\n", pwr);
    //return pwr;
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)output);
        // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

