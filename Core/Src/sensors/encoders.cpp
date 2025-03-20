#include "encoders.h"

EncodersData encoders_data;

void StartEncodersTask(void *argument)
{
	encoders_data.active = false;

	bool status = false;

	Encoder *left = &encoders_data.left;
	Encoder *right = &encoders_data.right;

	uint64_t last_time = HAL_GetTick();

	while (1)
	{
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Read the encoder data
		status = (left->data_ready && right->data_ready) && (left->active && right->active);

		if (!status)
		{
			if (!left->active || !right->active)
			{
				// Update the encoder data structure
				encoders_data.active = false;
				encoders_data.data_ready = false;

				osDelay(100);
			}

			osDelay(10);
			continue;
		}

		// Mark the encoder data as not ready
		left->data_ready = false;
		right->data_ready = false;

		// Map the encoder data to the encoder data structure
		encoders_data.velocity = (left->velocity + right->velocity) / 2.0f;
		encoders_data.angular_velocity = (right->velocity - left->velocity) / WHEEL_DISTANCE;

		printf("Encoders: %.4f %.4f\n", encoders_data.velocity, encoders_data.angular_velocity);

		// Update the encoder data structure
		encoders_data.active = true;
		encoders_data.data_ready = true;

		osDelay(10);
	}
}

// Measurement function
EKF::MeasurementVector h_encoders(const EKF::StateVector &x)
{
	EKF::MeasurementVector v(KALMAN_ENCODERS_MEASUREMENT_SIZE);

	v << x[6], // x'
		x[7],  // y'
		x[9];  // ψ'

	return v;
}

// Jacobian of measurement function
EKF::MeasurementJacobian H_encoders(const EKF::StateVector &x)
{
	EKF::MeasurementJacobian H = EKF::MeasurementJacobian::Zero(KALMAN_ENCODERS_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 6) = 1; // ∂x'/∂x'
	H(1, 7) = 1; // ∂y'/∂y'
	H(2, 9) = 1; // ∂ψ'/∂ψ'

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_encoders = Eigen::DiagonalMatrix<float, KALMAN_ENCODERS_MEASUREMENT_SIZE>{
	1.0e-4f, // x'
	1.0e-4f, // y'
	2.5e-5f	 // ψ' (yaw rate)
};

EKF::MeasurementVector encodersMeasurement(const EKF::StateVector &x)
{
	float forward_velocity = encoders_data.velocity;
	float angular_velocity = encoders_data.angular_velocity;

	// Compute x and y velocities
	float theta = robot.orientation[2];

	float v_x = -forward_velocity * sin(theta);
	float v_y = forward_velocity * cos(theta);

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_ENCODERS_MEASUREMENT_SIZE);
	z << v_x, v_y, angular_velocity;

	// Update the state estimate
	return z;
}

bool encodersDataReady()
{
	return encoders_data.data_ready;
}

void encodersDataConsume()
{
	encoders_data.data_ready = false;
}

Sensor encoders = {
	.h = h_encoders,
	.H = H_encoders,
	.R = R_encoders,
	.z = encodersMeasurement,
	.ready = encodersDataReady,
	.consume = encodersDataConsume};