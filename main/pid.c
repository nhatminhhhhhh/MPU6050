#include <pid.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <mpu.h>

float mpu_state = 0.00;
double output = 0;
float error=0.0;
float LastError = 0.0;
float mpu_set = 0.00;
float derivative = 0.00;
float integral = 0.0;
int pwr = 0;

unsigned int lastTimeReadMPU;
unsigned int lastTimeCalPID;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

#define TimeCalPID (float)(100.0) //ms
#define DeltaTime ( float)(TimeCalPID/1000) //s
#define TimeDelay (5)
#define MaxOutput (255.0)
#define MinOutput (0.0)
#define CONSTRAIN(x, low, high)  ((x)<(low)?(low):((x)>(high)?(high):(x)))


int pid_calculate() {
    if ( (esp_timer_get_time() - lastTimeReadMPU) >= 10 ) {
        mpu_state = mpu_read();
        lastTimeCalPID = esp_timer_get_time();
    }
    if ( (esp_timer_get_time() - lastTimeCalPID) >= TimeCalPID ) { //>= TimeCalPID * 1000
        error = mpu_set - mpu_state;
        integral += (error * DeltaTime);
        integral = CONSTRAIN(integral, -50, 50);
        derivative = (error - LastError) / DeltaTime;
        derivative = CONSTRAIN(derivative, -50, 50);

        output = Kp * error + Ki * error * DeltaTime + Kd * derivative;
        LastError = error;
        lastTimeCalPID = esp_timer_get_time();
    }

    pwr = (int)fabs(output);
    pwr = CONSTRAIN(output, -90, 90);
    // Debugging output
    //printf("MPU State: %.2f, Setpoint: %.2f, Error: %.2f, Output: %.2f\n", mpu_state, mpu_set, error, output);
    //printf("Output PID: %.2f\n", pwr);
    return pwr;
        // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)output);
        // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}