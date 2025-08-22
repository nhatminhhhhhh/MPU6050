
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
	// ESP_ERROR_CHECK(esp_netif_init());
	// ESP_ERROR_CHECK(esp_event_loop_create_default());
	 
	// ESP_LOGI(TAG, "Error reset: %d", esp_reset_reason());
    // ESP_LOGI(TAG, "Starting MPU...");
	mpu_init();
	while (1) {
		mpu_read();
	}
}
