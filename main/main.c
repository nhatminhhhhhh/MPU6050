
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <mpu.h>
#include <driver/uart.h>
#include <esp_system.h>
#include <driver/ledc.h>
#include <pid.h>

void app_main(void)
{
	// LEDC PWM setup for GPIO 17
	ledc_timer_config_t ledc_timer = {
		.speed_mode       = LEDC_HIGH_SPEED_MODE,
		.timer_num        = LEDC_TIMER_0,
		.duty_resolution  = LEDC_TIMER_8_BIT,
		.freq_hz          = 5000,
		.clk_cfg          = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel = {
		.speed_mode     = LEDC_HIGH_SPEED_MODE,
		.channel        = LEDC_CHANNEL_0,
		.timer_sel      = LEDC_TIMER_0,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = 17,
		.duty           = 0,
		.hpoint         = 0
	};
	ledc_channel_config(&ledc_channel);
	
	mpu_init();

	float static_data = 0.60;

	while (1) {
		float roll = mpu_read();
		printf(">Static Data: %.2f,Roll Angle: %.2f\n", static_data, roll);

		// Example: fade LED brightness up and down
		 //ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, pid_calculate());
		 //ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
		 //printf("Output PWM: %d\n", pid_calculate());
		
		//vTaskDelay(pdMS_TO_TICKS(10));
	}
}
