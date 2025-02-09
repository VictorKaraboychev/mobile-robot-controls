#include "sensors.h"

extern osMutexId_t spi1MutexHandle;

// LSM6DSO
LSM6DSO_Object_t lsm6dso;

int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

void StartImuTask(void *argument)
{
	lsm6dso.Ctx.handle = &hspi1;
	lsm6dso.Ctx.write_reg = Write_LSM6DSO;
	lsm6dso.Ctx.read_reg = Read_LSM6DSO;
	lsm6dso.Ctx.mdelay = HAL_Delay;

	lsm6dso.IO.BusType = LSM6DSO_SPI_4WIRES_BUS;

	uint8_t id = 0, rst = 0;

	// Reset the IMU
	lsm6dso_reset_set(&lsm6dso.Ctx, PROPERTY_ENABLE);
	do
	{
		lsm6dso_reset_get(&lsm6dso.Ctx, &rst);
	} while (rst);

	// Read the IMU ID
	do
	{
		LSM6DSO_ReadID(&lsm6dso, &id);
		osDelay(100);
	} while (id != LSM6DSO_ID);

	// Initialize the IMU
	LSM6DSO_Init(&lsm6dso);

	// Accelerometer configuration
	LSM6DSO_ACC_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
	LSM6DSO_ACC_SetFullScale(&lsm6dso, 4);
	LSM6DSO_ACC_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_20);
	LSM6DSO_ACC_Enable(&lsm6dso);

	// Gyroscope configuration
	LSM6DSO_GYRO_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE);
	LSM6DSO_GYRO_SetFullScale(&lsm6dso, 500);
	LSM6DSO_GYRO_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);
	LSM6DSO_GYRO_Enable(&lsm6dso);

	osDelay(10);

	// Accelerometer and gyroscope scale factors
	float acc_scale = GRAVITY / 1000.0f;
	float gyro_scale = DEG_TO_RAD / 1000.0f;

	LSM6DSO_Axes_t acc, gyro;
	bool status = false;

	// Calibrate the IMU
	uint16_t samples = 1000;
	Vector acc_bias(3), gyro_bias(3);

	for (uint16_t i = 0; i < samples; i++)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		acc_bias += -Vector{(float)acc.x, (float)acc.y, (float)acc.z};
		gyro_bias += Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z};

		osDelay(2);
	}

	acc_bias *= (acc_scale / (float)samples);
	gyro_bias *= (gyro_scale / (float)samples);

	*acc_bias.z += GRAVITY; // Subtract gravity from the z-axis

	while (1)
	{
		// printf("LSM6DSO ID: 0x%02X\n", id);

		// LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		// LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		// printf("Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n",
		//        (float)(acc.x * acc_scale),
		//        (float)(acc.y * acc_scale),
		//        (float)(acc.z * acc_scale),
		//        (float)(gyro.x * gyro_scale),
		//        (float)(gyro.y * gyro_scale),
		//        (float)(gyro.z * gyro_scale));

		status = (LSM6DSO_ACC_GetAxes(&lsm6dso, &acc) == LSM6DSO_OK) && (LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro) == LSM6DSO_OK);

		// Check if the IMU is active
		if (!status || id != LSM6DSO_ID)
		{
			imu_data.active = false;

			osDelay(100);
			continue;
		}

		// Map the IMU data to the IMU data structure
		imu_data.acceleration = -Vector{(float)acc.x, (float)acc.y, (float)acc.z} * acc_scale - acc_bias;
		imu_data.angular_velocity = Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z} * gyro_scale - gyro_bias;

		imu_data.active = true;

		// ComputeStatisticsRecursive(&stats, 1000, *imu_data.angular_velocity.z);

		// Print biases
		// printf("ID: 0x%02X Acceleration: %.4f %.4f %.4f Gyroscope: %.4f %.4f %.4f\n", id, *acc_bias.x, *acc_bias.y, *acc_bias.z, *gyro_bias.x, *gyro_bias.y, *gyro_bias.z);

		// printf("ID: 0x%02X Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n", id, *imu_data.acceleration.x, *imu_data.acceleration.y, *imu_data.acceleration.z, *imu_data.angular_velocity.x, *imu_data.angular_velocity.y, *imu_data.angular_velocity.z);

		osDelay(100);
	}
}

// LIS2MDL

LIS2MDL_Object_t lis2mdl;

int32_t Write_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS2MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

void StartMagTask(void *argument)
{
	lis2mdl.Ctx.handle = &hspi1;
	lis2mdl.Ctx.write_reg = Write_LIS2MDL;
	lis2mdl.Ctx.read_reg = Read_LIS2MDL;
	lis2mdl.Ctx.mdelay = HAL_Delay;

	lis2mdl.IO.BusType = LIS2MDL_SPI_4WIRES_BUS;

	uint8_t id = 0, rst = 0;

	// Reset the magnetometer
	lis2mdl_reset_set(&lis2mdl.Ctx, PROPERTY_ENABLE);
	do
	{
		lis2mdl_reset_get(&lis2mdl.Ctx, &rst);
	} while (rst);

	// Set the SPI mode to 4-wire
	lis2mdl_spi_mode_set(&lis2mdl.Ctx, LIS2MDL_SPI_4_WIRE);

	// Read the magnetometer ID
	do
	{
		LIS2MDL_ReadID(&lis2mdl, &id);
		osDelay(100);
	} while (id != LIS2MDL_ID);

	// Initialize the magnetometer
	LIS2MDL_Init(&lis2mdl);

	// Magnetometer configuration
	LIS2MDL_MAG_SetOutputDataRate(&lis2mdl, 100.0f);
	LIS2MDL_MAG_SetFullScale(&lis2mdl, 50);
	LIS2MDL_MAG_Set_Power_Mode(&lis2mdl, LIS2MDL_HIGH_RESOLUTION);
	LIS2MDL_MAG_Enable(&lis2mdl);

	osDelay(10);

	// Magnetometer scale factor
	float mag_scale = 1.5f / 1000.0f;

	LIS2MDL_Axes_t mag;
	bool status = false;

	// Calibrate the magnetometer


	while (1)
	{
		// printf("LIS2MDL ID: 0x%02X\n", id);

		status = (LIS2MDL_MAG_GetAxes(&lis2mdl, &mag) == LIS2MDL_OK);

		// Check if the IMU is active
		if (!status || id != LIS2MDL_ID)
		{
			imu_data.active = false;

			osDelay(100);
			continue;
		}

		// printf("Magnetometer: %.2f %.2f %.2f\n",
		//        (float)(mag.x * mag_scale),
		//        (float)(mag.y * mag_scale),
		//        (float)(mag.z * mag_scale));

		float roll, pitch, yaw;

		roll = atan2(mag.y, mag.z);
		pitch = atan2(-mag.x, sqrt(mag.y * mag.y + mag.z * mag.z));
		yaw = atan2(mag.z, mag.x);

		mag_data.magnetic_orientation = Vector{roll, pitch, yaw};

		imu_data.active = true;

		// printf("Roll: %.1f Pitch: %.1f Yaw: %.1f\n", roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

		osDelay(100);
	}
}

// Encoders

void StartEncodersTask(void *argument)
{

	bool status = false;

	while (1)
	{

		if (!status)
		{
			encoders_data.active = false;

			osDelay(100);
			continue;
		}

		encoders_data.active = true;

		osDelay(100);
	}
}