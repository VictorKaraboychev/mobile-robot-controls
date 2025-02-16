#include "barometer.h"

extern osMutexId_t spi1MutexHandle;

LPS22HH_Object_t lps22hh;

BarometerData barometer_data;

int32_t Write_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

int32_t Read_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

void StartBarometerTask(void *argument)
{
	lps22hh.Ctx.handle = &hspi1;
	lps22hh.Ctx.write_reg = Write_LPS22HH;
	lps22hh.Ctx.read_reg = Read_LPS22HH;
	lps22hh.Ctx.mdelay = HAL_Delay;

	lps22hh.IO.BusType = LPS22HH_SPI_4WIRES_BUS;

	barometer_data.active = false;

	uint8_t id = 0, rst = 0;

	// Reset the barometer
	lps22hh_reset_set(&lps22hh.Ctx, PROPERTY_ENABLE);
	do
	{
		lps22hh_reset_get(&lps22hh.Ctx, &rst);
	} while (rst);

	// Read the barometer ID
	do
	{
		LPS22HH_ReadID(&lps22hh, &id);
		printf("Barometer ID: 0x%02X\n", id);
		osDelay(100);
	} while (id != LPS22HH_ID);

	// Initialize the barometer
	LPS22HH_Init(&lps22hh);

	// Barometer configuration
	LPS22HH_PRESS_SetOutputDataRate(&lps22hh, 200.0f);
	LPS22HH_Set_Filter_Mode(&lps22hh, LPS22HH_LPF_ODR_DIV_9);
	LPS22HH_PRESS_Enable(&lps22hh);

	// Temperature configuration
	LPS22HH_TEMP_SetOutputDataRate(&lps22hh, 25.0f);
	LPS22HH_TEMP_Enable(&lps22hh);

	uint16_t samples = 200;
	float pressure, temperature;

	for (uint16_t i = 0; i < samples; i++)
	{
		LPS22HH_PRESS_GetPressure(&lps22hh, &pressure);

		barometer_data.reference_pressure += pressure;

		osDelay(5);
	}

	barometer_data.reference_pressure /= (float)samples;
	barometer_data.reference_altitude = 44330.0f * (1.0f - powf(barometer_data.reference_pressure / SEA_LEVEL_PRESSURE, 0.1903f));

	bool status = false;

	while (1)
	{
		// Read the barometer data
		status = (LPS22HH_PRESS_GetPressure(&lps22hh, &pressure) == LPS22HH_OK) && (LPS22HH_TEMP_GetTemperature(&lps22hh, &temperature) == LPS22HH_OK);

		// Check if the barometer is active
		if (!status || id != LPS22HH_ID)
		{
			// Update the barometer data structure
			barometer_data.active = false;
			barometer_data.data_ready = false;

			printf("Barometer not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		// Update the barometer data structure
		barometer_data.pressure = pressure;
		barometer_data.temperature = temperature;

		// Compute the altitude
		barometer_data.altitude = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.1903f)) - barometer_data.reference_altitude;

		// Update the barometer data structure
		barometer_data.active = true;
		barometer_data.data_ready = true;

		osDelay(5); // 200 Hz
	}
}

// Measurement function
EKF::MeasurementVector h_barometer(const EKF::StateVector &x)
{
	EKF::MeasurementVector v(KALMAN_BAROMETER_MEASUREMENT_SIZE);

	v << x[2]; // z

	return v;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_barometer(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_BAROMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 2) = 1; // ∂z/∂z

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_barometer = Eigen::DiagonalMatrix<float, KALMAN_BAROMETER_MEASUREMENT_SIZE>{{
	1.0e-2f // z
}};

EKF::MeasurementVector barometerMeasurement(const EKF::StateVector &x)
{
	float altitude = barometer_data.altitude;

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_BAROMETER_MEASUREMENT_SIZE);
	z << altitude;

	// Update the state estimate
	return z;
}

bool barometerDataReady()
{
	return barometer_data.data_ready;
}

Sensor barometer = {
	.h = h_barometer,
	.H = H_barometer,
	.R = R_barometer,
	.z = barometerMeasurement,
	.ready = barometerDataReady};