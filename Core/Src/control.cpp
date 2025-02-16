#include "control.h"

// State transition function
EKF::ProcessVector f(const EKF::StateVector &x, const EKF::ControlVector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	float phi = x[3] + x[9] * dt;
	float theta = x[4] + x[10] * dt;
	float psi = x[5] + x[11] * dt;

	// Constrain angles to [-π, π]
	if (abs(phi) > M_PI)
	{
		phi -= copysignf(M_TWOPI, phi);
	}
	if (abs(theta) > M_PI)
	{
		theta -= copysignf(M_TWOPI, theta);
	}
	if (abs(psi) > M_PI)
	{
		psi -= copysignf(M_TWOPI, psi);
	}

	return EKF::StateVector{
		x[0] + x[6] * dt + x[12] * dt2, // x = x + x' * Δt + 0.5 * x'' * Δt^2
		x[1] + x[7] * dt + x[13] * dt2, // y = y + y' * Δt + 0.5 * y'' * Δt^2
		x[2] + x[8] * dt + x[14] * dt2, // z = z + z' * Δt + 0.5 * z'' * Δt^2
		phi,							// φ = φ + φ' * Δt
		theta,							// θ = θ + θ' * Δt
		psi,							// ψ = ψ + ψ' * Δt
		x[6] + x[12] * dt,				// x' = x' + x'' * Δt
		x[7] + x[13] * dt,				// y' = y' + y'' * Δt
		x[8] + x[14] * dt,				// z' = z' + z'' * Δt
		x[9],							// φ' = φ'
		x[10],							// θ' = θ'
		x[11],							// ψ' = ψ'
		x[12],							// x'' = x''
		x[13],							// y'' = y''
		x[14]							// z'' = z''
	};
}

// Jacobian of state transition function
EKF::ProcessJacobian F(const EKF::StateVector &x, const EKF::ControlVector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	EKF::ProcessJacobian F = EKF::ProcessJacobian::Identity();

	F(0, 6) = dt;	// ∂x/∂x'
	F(0, 12) = dt2; // ∂x/∂x''

	F(1, 7) = dt;	// ∂y/∂y'
	F(1, 13) = dt2; // ∂y/∂y''

	F(2, 8) = dt;	// ∂z/∂z'
	F(2, 14) = dt2; // ∂z/∂z''

	F(4, 10) = dt; // ∂φ/∂φ'

	F(3, 9) = dt; // ∂θ/∂θ'

	F(5, 11) = dt; // ∂ψ/∂ψ'

	F(6, 12) = dt; // ∂x'/∂x''

	F(7, 13) = dt; // ∂y'/∂y''

	F(8, 14) = dt; // ∂z'/∂z''

	return F;
}

// Process noise covariance
EKF::ProcessCovariance Q = Eigen::DiagonalMatrix<float, KALMAN_STATE_SIZE>{
	1.0e-3f, // x 0
	1.0e-3f, // y 1
	1.0e-3f, // z 2

	5.0e-4f, // φ 3
	5.0e-4f, // θ 4
	5.0e-4f, // ψ 5

	1.0e-4f, // x' 6
	1.0e-4f, // y' 7
	1.0e-4f, // z' 8

	5.0e-5f, // φ' 9
	5.0e-5f, // θ' 10
	5.0e-5f, // ψ' 11

	1.0e-5f, // x'' 12
	1.0e-5f, // y'' 13
	1.0e-5f, // z'' 14
};

// --- Accelerometer ---

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
	EKF::MeasurementJacobian H(KALMAN_ACCELEROMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

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

void UpdateAccelerometer(const Eigen::Vector3f &acceleration)
{
	// Magnitude of the acceleration vector
	float magnitude = acceleration.norm();

	// Compute pitch and roll
	Eigen::Vector3f orientation = robot.orientation;

	// If the magnitude of the acceleration vector is close to gravity, the pitch and roll angles can be calculated using the accelerometer
	if (abs(magnitude - GRAVITY) < 0.1f)
	{
		orientation[0] = atan2(acceleration[1], -acceleration[2]);
		orientation[1] = atan2(-acceleration[0], hypot(acceleration[1], acceleration[2]));
	}

	// Rotate the acceleration vector to the world frame
	Eigen::Quaternionf q = euler2Quaternion(orientation[0], orientation[1], orientation[2]);

	Eigen::Vector3f world_acceleration = q.matrix() * acceleration;

	// Subtract gravity from the z-axis
	world_acceleration[2] += GRAVITY;

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_ACCELEROMETER_MEASUREMENT_SIZE);
	z << orientation[0], orientation[1], world_acceleration[0], world_acceleration[1], world_acceleration[2];

	// Update the state estimate
	ekf.asyncUpdate(z, h_accelerometer, H_accelerometer, R_accelerometer);
}

// ---Gyroscope---

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
	EKF::MeasurementJacobian H(KALMAN_GYROSCOPE_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

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

void UpdateGyroscope(const Eigen::Vector3f &angular_velocity)
{
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
	// EKF::MeasurementVector z(KALMAN_GYROSCOPE_MEASUREMENT_SIZE);

	// Update the state estimate
	// ekf.asyncUpdate(z, h_gyroscope, H_gyroscope, R_gyroscope);
}

// ---Magnetometer---

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
	EKF::MeasurementJacobian H(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE, KALMAN_STATE_SIZE);

	H(0, 5) = 1; // ∂ψ/∂ψ

	return H;
}

// Measurement noise covariance
EKF::MeasurementCovariance R_magnetometer = Eigen::DiagonalMatrix<float, KALMAN_MAGNETOMETER_MEASUREMENT_SIZE>{{
	1.0e-4f // ψ (yaw)
}};

void UpdateMagnetometer(const Eigen::Vector3f &orientation)
{
	// Update the state vector
	EKF::MeasurementVector z(KALMAN_MAGNETOMETER_MEASUREMENT_SIZE);
	z << orientation[2]; // ψ (yaw)

	// Update the state estimate
	// ekf.asyncUpdate(z, h_magnetometer, H_magnetometer, R_magnetometer);
}

// ---Encoders---

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

void UpdateEncoders(const float &forward_velocity, const float &angular_velocity)
{
	// Compute x and y velocities
	float theta = robot.orientation[2];

	float v_x = -forward_velocity * sin(theta);
	float v_y = forward_velocity * cos(theta);

	// Update the state vector
	EKF::MeasurementVector z(KALMAN_ENCODERS_MEASUREMENT_SIZE);
	z << v_x, v_y, angular_velocity;

	// Update the state estimate
	// ekf.asyncUpdate(z, h_encoders, H_encoders, R_encoders);
}

EKF ekf(f, F, Q);
RobotState robot;

void StartFusionTask(void *argument)
{
	// Set the initial state
	EKF::StateVector x = EKF::StateVector::Zero();
	EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.01f;
	ekf.initialize(x, P);

	osDelay(2000); // Wait for the sensors to initialize

	uint64_t last_time = HAL_GetTick();
	osDelay(10); // 100 Hz

	while (true)
	{
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Create the control vector
		EKF::ControlVector u {delta_time};

		// Predict the state
		ekf.predict(u);

		// Update the state from the state vector
		EKF::StateVector state = ekf.getState();

		robot.position = Eigen::Vector3f{state[0], state[1], state[2]};
		robot.velocity = Eigen::Vector3f{state[6], state[7], state[8]};
		robot.acceleration = Eigen::Vector3f{state[12], state[13], state[14]};

		robot.orientation = Eigen::Vector3f{state[3], state[4], state[5]};
		robot.angular_velocity = Eigen::Vector3f{state[9], state[10], state[11]};

		osDelay(10); // 100 Hz
	}
}

void StartControlTask(void *argument)
{
	while (true)
	{
		Eigen::Vector2f position = robot.position.head<2>();
		Eigen::Vector2f target{0, 0}; // GetTarget();

		// Calculate the curvature
		float curvature = PurePursuit<float>::CalculateCurvature(position, robot.orientation[2], target);

		// Calculate the left and right wheel velocities
		float left_velocity = TARGET_SPEED * (1 - curvature * WHEEL_DISTANCE / 2.0f);
		float right_velocity = TARGET_SPEED * (1 + curvature * WHEEL_DISTANCE / 2.0f);

		// Limit the wheel velocities to the maximum speed
		float max_velocity = std::max(abs(left_velocity), abs(right_velocity));

		// If the maximum velocity is greater than the maximum speed (maintain the ratio)
		if (max_velocity > MAX_SPEED)
		{
			left_velocity *= MAX_SPEED / max_velocity;
			right_velocity *= MAX_SPEED / max_velocity;
		}

		osDelay(10); // 100 Hz
	}
}
