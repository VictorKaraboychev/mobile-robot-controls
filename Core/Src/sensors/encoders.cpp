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
		// status = (Read_Encoders(&left->pulses, &right->pulses) == HAL_OK); // TODO: Implement the Read_Encoder function

		if (!status)
		{
			// Update the encoder data structure
			encoders_data.active = false;
			encoders_data.data_ready = false;

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
	EKF::MeasurementJacobian H(KALMAN_ENCODERS_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

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

Sensor encoders = {
	.measurement_function = h_encoders,
	.measurement_jacobian_function = H_encoders,
	.measurement_covariance = R_encoders,
	.measurement = encodersMeasurement,
	.measurement_ready = encodersDataReady};