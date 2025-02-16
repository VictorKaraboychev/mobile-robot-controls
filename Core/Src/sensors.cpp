#include "sensors.h"

extern osMutexId_t spi1MutexHandle;

// LSM6DSO
LSM6DSO_Object_t lsm6dso;
IMUData imu_data;

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
		printf("IMU ID: 0x%02X\n", id);
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
	Eigen::Vector3f acc_bias, gyro_bias;

	for (uint16_t i = 0; i < samples; i++)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		acc_bias += Eigen::Vector3f{(float)acc.x, (float)acc.y, (float)acc.z};
		gyro_bias += Eigen::Vector3f{(float)gyro.x, (float)gyro.y, (float)gyro.z};

		osDelay(2);
	}

	acc_bias *= (acc_scale / (float)samples);
	gyro_bias *= (gyro_scale / (float)samples);

	acc_bias[2] += GRAVITY; // Subtract gravity from the z-axis

	printf("Accelerometer Bias: %.2f %.2f %.2f Gyroscope Bias: %.4f %.4f %.4f\n", acc_bias[0], acc_bias[1], acc_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	while (1)
	{
		// Read the IMU data
		status = (LSM6DSO_ACC_GetAxes(&lsm6dso, &acc) == LSM6DSO_OK) && (LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro) == LSM6DSO_OK);

		// Check if the IMU is active
		if (!status || id != LSM6DSO_ID)
		{
			imu_data.active = false;

			printf("IMU not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		// Map the IMU data to the IMU data structure
		imu_data.acceleration = -(Eigen::Vector3f{(float)acc.x, (float)acc.y, (float)acc.z} * acc_scale - acc_bias);
		imu_data.angular_velocity = Eigen::Vector3f{(float)gyro.x, (float)gyro.y, (float)gyro.z} * gyro_scale - gyro_bias;

		imu_data.active = true;

		// printf("ID: 0x%02X Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n", id, *imu_data.acceleration.x, *imu_data.acceleration.y, *imu_data.acceleration.z, *imu_data.angular_velocity.x, *imu_data.angular_velocity.y, *imu_data.angular_velocity.z);

		// Update the EKF with the IMU data
		UpdateAccelerometer(imu_data.acceleration);
		UpdateGyroscope(imu_data.angular_velocity);

		osDelay(10); // 100 Hz
	}
}

// LIS2MDL

LIS2MDL_Object_t lis2mdl;
MagData mag_data;

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
		printf("Magnetometer ID: 0x%02X\n", id);
		osDelay(100);
	} while (id != LIS2MDL_ID);

	// Initialize the magnetometer
	LIS2MDL_Init(&lis2mdl);
	lis2mdl_spi_mode_set(&lis2mdl.Ctx, LIS2MDL_SPI_4_WIRE);

	// Magnetometer configuration
	LIS2MDL_MAG_SetOutputDataRate(&lis2mdl, 100.0f);
	LIS2MDL_MAG_SetFullScale(&lis2mdl, 50);
	LIS2MDL_MAG_Set_Power_Mode(&lis2mdl, LIS2MDL_HIGH_RESOLUTION);
	LIS2MDL_MAG_Enable(&lis2mdl);

	osDelay(10);

	// Magnetometer scale factor
	float mag_scale = 1.5f / 1000.0f;

	LIS2MDL_Axes_t mag;
	float roll, pitch, yaw;
	bool status = false;

	// Calibrate the magnetometer
	uint16_t samples = 1000;

	Eigen::Matrix3f soft_iron;
	Eigen::Vector3f hard_iron;

	for (uint16_t i = 0; i < samples; i++)
	{
		LIS2MDL_MAG_GetAxes(&lis2mdl, &mag);

		soft_iron(0, 0) += mag.x * mag.x;
		soft_iron(0, 1) += mag.x * mag.y;
		soft_iron(0, 2) += mag.x * mag.z;
		soft_iron(1, 1) += mag.y * mag.y;
		soft_iron(1, 2) += mag.y * mag.z;
		soft_iron(2, 2) += mag.z * mag.z;

		hard_iron += Eigen::Vector3f{(float)mag.x, (float)mag.y, (float)mag.z};

		osDelay(5);
	}

	soft_iron /= (float)samples;
	hard_iron /= (float)samples;

	// lis2mdl_mag_user_offset_set(&lis2mdl.Ctx, (int16_t)hard_iron.x, (int16_t)hard_iron.y, (int16_t)hard_iron.z);

	while (1)
	{
		// Read the magnetometer data
		status = (LIS2MDL_MAG_GetAxes(&lis2mdl, &mag) == LIS2MDL_OK);

		// Check if the IMU is active
		if (!status || id != LIS2MDL_ID)
		{
			imu_data.active = false;

			printf("Magnetometer not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		// Compute the magnetic orientation
		roll = atan2(mag.y, mag.z);
		pitch = atan2(-mag.x, hypot(mag.y, mag.z));
		yaw = atan2(mag.z, mag.x);

		mag_data.magnetic_orientation = Eigen::Vector3f{roll, pitch, yaw};

		mag_data.active = true;

		// printf("ID: 0x%02X Roll: %.1f Pitch: %.1f Yaw: %.1f\n", id, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

		// Update the EKF with the magnetometer data
		// UpdateMagnetometer(mag_data.magnetic_orientation); TODO: Enable this when the magnetometer calibration is done and working

		osDelay(100);
	}
}

// Encoders

EncodersData encoders_data;

void StartEncodersTask(void *argument)
{
	bool status = false;

	Encoder *left = &encoders_data.left;
	Encoder *right = &encoders_data.right;

	uint64_t last_time = HAL_GetTick();

	while (1)
	{
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Read the encoder data
		// status = (Read_Encoders(&left->pulses, &right->pulses) == HAL_OK); // TODO: Implement the Read_Encoder function

		if (!status)
		{
			encoders_data.active = false;

			osDelay(100);
			continue;
		}

		// Compute the delta pulses
		int64_t delta_left = left->pulses - left->last_pulses;
		int64_t delta_right = right->pulses - right->last_pulses;

		// If the left encoder has made a full revolution handle the discontinuity
		if (abs(delta_left) > PULSE_PER_REVOLUTION / 2)
		{
			delta_left -= copysignf(PULSE_PER_REVOLUTION, delta_left);
		}

		// If the right encoder has made a full revolution
		if (abs(delta_right) > PULSE_PER_REVOLUTION / 2)
		{
			delta_right -= copysignf(PULSE_PER_REVOLUTION, delta_right);
		}

		// Compute the encoder velocity
		float left_velocity = delta_left * (DISTANCE_PER_PULSE / delta_time);
		float right_velocity = delta_right * (DISTANCE_PER_PULSE / delta_time);

		// Map the encoder data to the encoder data structure
		encoders_data.velocity = (left_velocity + right_velocity) / 2.0f;
		encoders_data.angular_velocity = (right_velocity - left_velocity) / WHEEL_DISTANCE;

		// Store the last encoder values
		left->last_pulses = left->pulses;
		right->last_pulses = right->pulses;

		encoders_data.active = true;

		// Update the EKF with the encoder data
		// UpdateEncoders(encoders_data.velocity, encoders_data.angular_velocity);

		osDelay(10);
	}
}