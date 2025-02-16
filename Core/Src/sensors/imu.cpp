#include "imu.h"

extern osMutexId_t spi1MutexHandle;

LSM6DSO_Object_t lsm6dso;

AccelerometerData accelerometer_data;
GyroscopeData gyroscope_data;

int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

void StartIMUTask(void *argument)
{
	lsm6dso.Ctx.handle = &hspi1;
	lsm6dso.Ctx.write_reg = Write_LSM6DSO;
	lsm6dso.Ctx.read_reg = Read_LSM6DSO;
	lsm6dso.Ctx.mdelay = HAL_Delay;

	lsm6dso.IO.BusType = LSM6DSO_SPI_4WIRES_BUS;

	accelerometer_data.active = false;
	gyroscope_data.active = false;

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
	const uint16_t samples = 1000;
	Eigen::Vector3f acc_bias, gyro_bias;

	for (uint16_t i = 0; i < samples; i++)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		acc_bias += -Eigen::Vector3f{(float)acc.x, (float)acc.y, (float)acc.z};
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
			// Update the IMU data structure
			accelerometer_data.active = false;
			accelerometer_data.data_ready = false;

			gyroscope_data.active = false;
			gyroscope_data.data_ready = false;

			printf("IMU not active, ID: 0x%02X\n", id);

			osDelay(100);
			continue;
		}

		// Map the IMU data to the IMU data structure
		accelerometer_data.acceleration = -Eigen::Vector3f{(float)acc.x, (float)acc.y, (float)acc.z} * acc_scale - acc_bias;
		gyroscope_data.angular_velocity = Eigen::Vector3f{(float)gyro.x, (float)gyro.y, (float)gyro.z} * gyro_scale - gyro_bias;

		// Update the IMU data structure
		accelerometer_data.active = true;
		accelerometer_data.data_ready = true;

		gyroscope_data.active = true;
		gyroscope_data.data_ready = true;

		osDelay(1); // 1000 Hz
	}
}

// Measurement function
EKF::MeasurementVector h_accelerometer(const EKF::StateVector &x)
{
	EKF::MeasurementVector v(KALMAN_ACCELEROMETER_MEASUREMENT_SIZE);

	v << x[3], // φ (roll)
		x[4],  // θ (pitch)
		x[12], // x''
		x[13], // y''
		x[14]; // z''

	return v;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_accelerometer(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_ACCELEROMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 3) = 1; // ∂φ/∂φ
	H(1, 4) = 1; // ∂θ/∂θ

	H(2, 12) = 1; // ∂x''/∂x''
	H(3, 13) = 1; // ∂y''/∂y''
	H(4, 14) = 1; // ∂z''/∂z''

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_accelerometer = Eigen::DiagonalMatrix<float, KALMAN_ACCELEROMETER_MEASUREMENT_SIZE>{
	5e-3f, // φ (roll)
	5e-3f, // θ (pitch)
	1e-4f, // x''
	1e-4f, // y''
	1e-4f  // z''
};

Eigen::Quaternionf euler2Quaternion(const float roll, const float pitch, const float yaw)
{
	Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

	Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;
	return q;
}

EKF::MeasurementVector accelerometerMeasurement(const EKF::StateVector &x)
{
	const Eigen::Vector3f &acceleration = accelerometer_data.acceleration;

	accelerometer_data.data_ready = false;

	// Magnitude of the acceleration vector
	float magnitude = acceleration.norm();

	// Compute pitch and roll
	Eigen::Vector3f orientation{
		x[3], // φ (roll)
		x[4], // θ (pitch)
		x[5]  // ψ (yaw)
	};

	// Rotate the acceleration vector to the world frame
	Eigen::Quaternionf q = euler2Quaternion(orientation[0], orientation[1], orientation[2]);
	Eigen::Vector3f world_acceleration = q.matrix().transpose() * acceleration;

	// Subtract gravity from the z-axis
	world_acceleration[2] += GRAVITY;

	// If the magnitude of the acceleration vector is close to gravity, the pitch and roll angles can be calculated using the accelerometer
	if (abs(magnitude - GRAVITY) < 0.2f)
	{
		orientation[0] = atan2(acceleration[1], -acceleration[2]);
		orientation[1] = atan2(-acceleration[0], hypot(acceleration[1], acceleration[2]));
	}

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_ACCELEROMETER_MEASUREMENT_SIZE);
	z << orientation[0], orientation[1], world_acceleration[0], world_acceleration[1], world_acceleration[2];

	return z;
}

bool accelerometerDataReady()
{
	return accelerometer_data.data_ready;
}

Sensor accelerometer = {
	.h = h_accelerometer,
	.H = H_accelerometer,
	.R = R_accelerometer,
	.z = accelerometerMeasurement,
	.ready = accelerometerDataReady};

// Measurement function
EKF::MeasurementVector h_gyroscope(const EKF::StateVector &x)
{
	EKF::MeasurementVector h(KALMAN_GYROSCOPE_MEASUREMENT_SIZE);

	h << x[9], // φ' (roll rate)
		x[10], // θ' (pitch rate)
		x[11]; // ψ' (yaw rate)

	return h;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_gyroscope(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_GYROSCOPE_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 9) = 1;  // ∂φ'/∂φ'
	H(1, 10) = 1; // ∂θ'/∂θ'
	H(2, 11) = 1; // ∂ψ'/∂ψ'

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_gyroscope = Eigen::DiagonalMatrix<float, KALMAN_GYROSCOPE_MEASUREMENT_SIZE>{
	5.0e-5f, // φ' (roll rate)
	5.0e-5f, // θ' (pitch rate)
	5.0e-5f	 // ψ' (yaw rate)
};

EKF::MeasurementVector gyroscopeMeasurement(const EKF::StateVector &x)
{
	const Eigen::Vector3f &angular_velocity = gyroscope_data.angular_velocity;

	// float s1 = sin(*robot.orientation.x);
	// float c1 = cos(*robot.orientation.x);

	// float c2 = cos(*robot.orientation.y);
	// float t2 = tan(*robot.orientation.y);

	// float p = *angular_velocity.x;
	// float q = *angular_velocity.y;
	// float r = *angular_velocity.z;

	// Vector world_angular_velocity = Vector{
	// 	p + (q * s1 + r * c1) * t2, // φ' = p + (q * s1 + r * c1) * t2
	// 	q * c1 - r * s1,			// θ' = q * c1 - r * s1
	// 	(q * s1 + r * c1) / c2		// ψ' = (q * s1 + r * c1) / c2
	// };

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_GYROSCOPE_MEASUREMENT_SIZE);

	// Update the state estimate
	return z;
}

bool gyroscopeDataReady()
{
	return gyroscope_data.data_ready;
}

Sensor gyroscope = {
	.h = h_gyroscope,
	.H = H_gyroscope,
	.R = R_gyroscope,
	.z = gyroscopeMeasurement,
	.ready = gyroscopeDataReady};