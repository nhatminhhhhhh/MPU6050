#include <mpu.h>


const char *TAG = "MPU_TEST";
float RateRoll = 0, RatePitch = 0, RateYaw = 0;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int RateCalibrationNumber = 0;
float AccX = 0, AccY = 0, AccZ = 0;
float AngleRoll = 0, AnglePitch = 0;
uint64_t LoopTimer = 0;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[2] = {0, 0};
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

esp_err_t i2c_write(uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
esp_err_t i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
	KalmanState=KalmanState+0.004*KalmanInput;
	KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
	float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
	KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
	KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
	Kalman1DOutput[0]=KalmanState; 
	Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(void) {
	uint8_t data[6];
	// Configure MPU6050
	i2c_write(0x1A, 0x05);
	i2c_write(0x1C, 0x10);
	// Read Accel
	i2c_read(0x3B, data, 6);
	int16_t AccXLSB = (data[0] << 8) | data[1];
	int16_t AccYLSB = (data[2] << 8) | data[3];
	int16_t AccZLSB = (data[4] << 8) | data[5];
	// Configure Gyro
	i2c_write(0x1B, 0x08);
	i2c_read(0x43, data, 6);
	int16_t GyroX = (data[0] << 8) | data[1];
	int16_t GyroY = (data[2] << 8) | data[3];
	int16_t GyroZ = (data[4] << 8) | data[5];
	RateRoll=(float)GyroX/65.5;
	RatePitch=(float)GyroY/65.5;
	RateYaw=(float)GyroZ/65.5;
	AccX=(float)AccXLSB/4096;
	AccY=(float)AccYLSB/4096;
	AccZ=(float)AccZLSB/4096;
	AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*180.0/M_PI;
	AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*180.0/M_PI;
}


void mpu_init(void) {
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	i2c_param_config(I2C_MASTER_NUM, &conf);
	i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
	i2c_write(0x6B, 0x00); // Wake up MPU6050
	vTaskDelay(pdMS_TO_TICKS(250)); // Allow time for MPU to wake up

	RateCalibrationRoll = 0;
	RateCalibrationPitch = 0;
	RateCalibrationYaw = 0;
	for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++) {
		gyro_signals();
		RateCalibrationRoll+=RateRoll;
		RateCalibrationPitch+=RatePitch;
		RateCalibrationYaw+=RateYaw;
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	RateCalibrationRoll/=2000;
	RateCalibrationPitch/=2000;
	RateCalibrationYaw/=2000;
	LoopTimer=esp_timer_get_time();
}
void mpu_read(void) {
	
	gyro_signals();
		RateRoll-=RateCalibrationRoll;
		RatePitch-=RateCalibrationPitch;
		RateYaw-=RateCalibrationYaw;
		kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
		KalmanAngleRoll=Kalman1DOutput[0]; 
		KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
		kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
		KalmanAnglePitch=Kalman1DOutput[0]; 
		KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
		printf("Roll Angle [°] %.2f Pitch Angle [°] %.2f\n", KalmanAngleRoll, KalmanAnglePitch);
		while ((esp_timer_get_time() - LoopTimer) < 4000);
		LoopTimer=esp_timer_get_time();

	// ESP_LOGI(TAG, "Rate Calibration: Roll: %f, Pitch: %f, Yaw: %f", 
	// 		 RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);
}

// void mpu() {
// 	xTaskCreate(mpu_read, "mpu_read_task", 2048, NULL, 5, NULL);
// 	//ESP_LOGI(TAG, "MPU task started");
// }