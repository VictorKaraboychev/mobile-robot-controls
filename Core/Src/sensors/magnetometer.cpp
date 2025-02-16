#include "magnetometer.h"

extern osMutexId_t spi1MutexHandle;

LIS2MDL_Object_t lis2mdl;
MagnetometerData magnetometer_data;

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

	magnetometer_data.active = false;

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
			// Update the magnetometer data structure
			magnetometer_data.active = false;
			magnetometer_data.data_ready = false;

			printf("Magnetometer not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		// Compute the magnetic orientation
		roll = atan2(mag.y, mag.z);
		pitch = atan2(-mag.x, hypot(mag.y, mag.z));
		yaw = atan2(mag.z, mag.x);

		magnetometer_data.magnetic_orientation = Eigen::Vector3f{roll, pitch, yaw};

		// Update the magnetometer data structure
		magnetometer_data.active = true;
		magnetometer_data.data_ready = true;

		// printf("ID: 0x%02X Roll: %.1f Pitch: %.1f Yaw: %.1f\n", id, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG);

		osDelay(100);
	}
}

// Measurement function
EKF::MeasurementVector h_magnetometer(const EKF::StateVector &x)
{
	EKF::MeasurementVector v(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE);

	v << x[5]; // ψ (yaw)

	return v;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_magnetometer(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 5) = 1; // ∂ψ/∂ψ

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_magnetometer = Eigen::DiagonalMatrix<float, KALMAN_MAGNETOMETER_MEASUREMENT_SIZE>{{
	1.0e-4f // ψ (yaw)
}};

EKF::MeasurementVector magnetometerMeasurement(const EKF::StateVector &x)
{
	float yaw = magnetometer_data.magnetic_orientation[2];

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE);
	z << yaw; // ψ (yaw)

	// Update the state estimate
	return z;
}

bool magnetometerDataReady()
{
	return magnetometer_data.data_ready;
}

Sensor magnetometer = {
	.h = h_magnetometer,
	.H = H_magnetometer,
	.R = R_magnetometer,
	.z = magnetometerMeasurement,
	.ready = magnetometerDataReady};