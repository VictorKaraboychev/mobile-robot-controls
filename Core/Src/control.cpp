#include "control.h"

#define KALMAN_STATE_SIZE 15

// State transition function
Vector f(const Vector &x, const Vector &u)
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

	return Vector{
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
Matrix F(const Vector &x, const Vector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	Matrix F = Matrix::Identity(KALMAN_STATE_SIZE);

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
Matrix Q = Matrix::Diagonal({
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
});

// --- Accelerometer ---

// Measurement function
Vector h_accelerometer(const Vector &x)
{
	return Vector{
		x[3], // φ (roll)
		x[4], // θ (pitch)

		x[12], // x''
		x[13], // y''
		x[14]  // z''
	};
}

// Jacobian of measurement function
Matrix H_accelerometer(const Vector &x)
{
	Matrix H(5, KALMAN_STATE_SIZE);

	H(0, 3) = 1; // ∂φ/∂φ
	H(1, 4) = 1; // ∂θ/∂θ

	H(2, 12) = 1; // ∂x''/∂x''
	H(3, 13) = 1; // ∂y''/∂y''
	H(4, 14) = 1; // ∂z''/∂z''

	return H;
}

// Measurement noise covariance
Matrix R_accelerometer = Matrix::Diagonal({5e-3f, 5e-3f, 1e-4f, 1e-4f, 1e-4f}); // acc_noise_density = 75 μg/√Hz (0.00073575 m/s^2/√Hz) ^ 2 / 1000 Hz

void UpdateAccelerometer(const Vector &acceleration)
{
	// Magnitude of the acceleration vector
	float magnitude = acceleration.magnitude();

	// Compute pitch and roll
	Vector orientation = robot.orientation;

	// If the magnitude of the acceleration vector is close to gravity, the pitch and roll angles can be calculated using the accelerometer
	if (abs(magnitude - GRAVITY) < 0.1f)
	{
		*orientation.x = atan2(*acceleration.y, -*acceleration.z);
		*orientation.y = atan2(-*acceleration.x, hypot(*acceleration.y, *acceleration.z));
	}

	// Rotate the acceleration vector to the world frame
	Vector world_acceleration = Matrix::Rotation3D(robot.orientation) * acceleration;

	// Subtract gravity from the z-axis
	*world_acceleration.z += GRAVITY;

	// Update the state vector
	Vector z = Vector{*orientation.x, *orientation.y, *world_acceleration.x, *world_acceleration.y, *world_acceleration.z};

	// Update the state estimate
	ekf.asyncUpdate(z, h_accelerometer, H_accelerometer, R_accelerometer);
}

// ---Gyroscope---

// Measurement function
Vector h_gyroscope(const Vector &x)
{
	return Vector{
		x[9],  // θ' (pitch rate)
		x[10], // φ' (roll rate)
		x[11]  // ψ' (yaw rate)
	};
}

// Jacobian of measurement function
Matrix H_gyroscope(const Vector &x)
{
	Matrix H(3, KALMAN_STATE_SIZE);

	H(0, 9) = 1;  // ∂θ'/∂θ'
	H(1, 10) = 1; // ∂φ'/∂φ'
	H(2, 11) = 1; // ∂ψ'/∂ψ'

	return H;
}

// Measurement noise covariance
Matrix R_gyroscope = Matrix::Diagonal({5.0e-5f, 5.0e-5f, 5.0e-5f}); // gyro_noise_density = 0.0038 °/s/√Hz (0.000066 rad/s/√Hz) ^ 2 / 1000 Hz

void UpdateGyroscope(const Vector &angular_velocity)
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
	// Vector z = Vector{*world_angular_velocity.x, *world_angular_velocity.y, *world_angular_velocity.z};

	// z.print();

	// Update the state estimate
	// ekf.asyncUpdate(z, h_gyroscope, H_gyroscope, R_gyroscope);
}

// ---Magnetometer---

// Measurement function
Vector h_magnetometer(const Vector &x)
{
	return Vector{
		x[5] // ψ
	};
}

// Jacobian of measurement function
Matrix H_magnetometer(const Vector &x)
{
	Matrix H(1, KALMAN_STATE_SIZE);

	H(0, 5) = 1; // ∂ψ/∂ψ

	return H;
}

// Measurement noise covariance
Matrix R_magnetometer = Matrix::Diagonal({4.0e-4f});

void UpdateMagnetometer(const Vector &orientation)
{
	// Update the state vector
	Vector z = Vector{*orientation.z};

	// Update the state estimate
	// ekf.asyncUpdate(z, h_magnetometer, H_magnetometer, R_magnetometer);
}

// ---Encoders---

// Measurement function
Vector h_encoders(const Vector &x)
{
	return Vector{
		x[6], // x'
		x[7], // y'
		x[9]  // ψ'
	};
}

// Jacobian of measurement function
Matrix H_encoders(const Vector &x)
{
	Matrix H(2, KALMAN_STATE_SIZE);

	H(0, 6) = 1; // ∂x'/∂x'
	H(1, 7) = 1; // ∂y'/∂y'
	H(2, 9) = 1; // ∂ψ'/∂ψ'

	return H;
}

// Measurement noise covariance
Matrix R_encoders = Matrix::Diagonal({1.0e-4f, 2.5e-5f});

void UpdateEncoders(const float &forward_velocity, const float &angular_velocity)
{
	// Compute x and y velocities
	float theta = *robot.orientation.z;

	float v_x = -forward_velocity * sin(theta);
	float v_y = forward_velocity * cos(theta);

	// Update the state vector
	Vector z = Vector{v_x, v_y, angular_velocity};

	// Update the state estimate
	// ekf.asyncUpdate(z, h_encoders, H_encoders, R_encoders);
}

ExtendedKalmanFilter ekf = ExtendedKalmanFilter(f, F, Q);
RobotState robot;

void StartFusionTask(void *argument)
{
	// Set the initial state
	Vector x(KALMAN_STATE_SIZE);
	Matrix P = Matrix::Identity(KALMAN_STATE_SIZE) * 0.01f;
	ekf.initialize(x, P);

	osDelay(2000); // Wait for the sensors to initialize

	uint64_t last_time = HAL_GetTick();
	osDelay(10); // 100 Hz

	while (true)
	{
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Predict the state
		ekf.predict(Vector{delta_time});

		// Update the state from the state vector
		Vector state = ekf.getState();

		robot.position = Vector{state[0], state[1], state[2]};
		robot.velocity = Vector{state[6], state[7], state[8]};
		robot.acceleration = Vector{state[12], state[13], state[14]};

		robot.orientation = Vector{state[3], state[4], state[5]};
		robot.angular_velocity = Vector{state[9], state[10], state[11]};

		osDelay(10); // 100 Hz
	}
}

void StartControlTask(void *argument)
{
	while (true)
	{

		// Vector target = Vector({0, 0}); // GetTarget();

		// // Calculate the curvature
		// float curvature = 0; // calculateCurvature(robot.position, robot.orientation, target);

		// // Calculate the left and right wheel velocities
		// float left_velocity = TARGET_SPEED * (1 - curvature * WHEEL_DISTANCE / 2.0f);
		// float right_velocity = TARGET_SPEED * (1 + curvature * WHEEL_DISTANCE / 2.0f);

		// // Limit the wheel velocities to the maximum speed
		// float max_velocity = std::max(abs(left_velocity), abs(right_velocity));

		// // If the maximum velocity is greater than the maximum speed (maintain the ratio)
		// if (max_velocity > MAX_SPEED)
		// {
		// 	left_velocity *= MAX_SPEED / max_velocity;
		// 	right_velocity *= MAX_SPEED / max_velocity;
		// }

		osDelay(10); // 100 Hz
	}
}
