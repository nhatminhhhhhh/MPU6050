
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <mpu.h>


void app_main(void)
{

	mpu_init();
	while (1) {
		float roll = mpu_read();
		printf("Roll Angle [°]: %.2f\n", roll);
	}
}
