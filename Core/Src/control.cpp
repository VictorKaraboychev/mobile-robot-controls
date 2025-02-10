#include "control.h"

// State transition function
Vector f(const Vector &x, const Vector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	return Vector{
		x[0] + x[3] * dt + x[6] * dt2, // x = x + v_x * Δt + 0.5 * a_x * Δt^2
		x[1] + x[4] * dt + x[7] * dt2, // y = y + v_y * Δt + 0.5 * a_y * Δt^2
		x[2] + x[5] * dt + x[8] * dt2, // θ = θ + ω * Δt + 0.5 * α * Δt^2
		x[3] + x[6] * dt,			   // v_x = v_x + a_x * Δt
		x[4] + x[7] * dt,			   // v_y = v_y + a_y * Δt
		x[5] + x[8] * dt,			   // ω = ω + α * Δt
		x[6],						   // a_x = a_x
		x[7],						   // a_y = a_y
		x[8]						   // α = α
	};
}

// Jacobian of state transition function
Matrix F(const Vector &x, const Vector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	Matrix F = Matrix::Identity(9);

	// Update non-identity terms
	F(0, 3) = dt;  // ∂x/∂vx
	F(0, 6) = dt2; // ∂x/∂ax

	F(1, 4) = dt;  // ∂y/∂vy
	F(1, 7) = dt2; // ∂y/∂ay

	F(2, 5) = dt;  // ∂θ/∂ω
	F(2, 8) = dt2; // ∂θ/∂α

	F(3, 6) = dt; // ∂vx/∂ax
	F(4, 7) = dt; // ∂vy/∂ay
	F(5, 8) = dt; // ∂ω/∂α

	return F;
}

// Process noise covariance
Matrix Q = Matrix::Diagonal({
	1.0e-3f, // x
	1.0e-3f, // y
	5.0e-4f, // θ
	1.0e-4f, // v_x
	1.0e-4f, // v_y
	5.0e-5f, // ω
	1.0e-5f, // a_x
	1.0e-5f, // a_y
	5.0e-6f	 // α
});

// ---IMU (accelerometer and gyroscope)---

// Measurement function
Vector h_imu(const Vector &x)
{
	float theta = x[2];
	float s = sin(theta);
	float c = cos(theta);

	return Vector{
		x[6] * c - x[7] * s, // a_x_body = a_x * cos(θ) - a_y * sin(θ)
		x[6] * s + x[7] * c, // a_y_body = a_x * sin(θ) + a_y * cos(θ)
		x[5]				 // ω
	};
}

// Jacobian of measurement function
Matrix H_imu(const Vector &x)
{
	float theta = x[2];
	float s = sin(theta);
	float c = cos(theta);

	Matrix H(3, 9);

	H(0, 6) = c;					// ∂a_x_body/∂a_x
	H(0, 7) = -s;					// ∂a_x_body/∂a_y
	H(0, 2) = -x[6] * s - x[7] * c; // ∂a_x_body/∂θ

	H(1, 6) = s;				   // ∂a_y_body/∂a_x
	H(1, 7) = c;				   // ∂a_y_body/∂a_y
	H(1, 2) = x[6] * c - x[7] * s; // ∂a_y_body/∂θ

	H(2, 5) = 1; // ∂ω/∂w

	return H;
}

// Measurement noise covariance

// acc_noise_density = 75 μg/√Hz (0.00073575 m/s^2/√Hz) ^ 2 / 1000 Hz
// gyro_noise_density = 0.0038 °/s/√Hz (0.000066 rad/s/√Hz) ^ 2 / 1000 Hz
Matrix R_imu = Matrix::Diagonal({7.5e-4f, 7.5e-4f, 5.0e-5f});

// ---Magnetometer---

// Measurement function
Vector h_mag(const Vector &x)
{
	return Vector{
		x[2] // θ
	};
}

// Jacobian of measurement function
Matrix H_mag(const Vector &x)
{
	Matrix H(1, 9);

	H(0, 2) = 1; // ∂θ/∂θ

	return H;
}

// Measurement noise covariance
Matrix R_mag = Matrix::Diagonal({4.0e-4f});

// ---Encoders---

// Measurement function
Vector h_enc(const Vector &x)
{
	float theta = x[2];
	float s = sin(theta);
	float c = cos(theta);

	return Vector{
		x[3] * c - x[4] * s, // v_fwd = v_x * cos(θ) - v_y * sin(θ)
		x[5]				 // ω = ω
	};
}

// Jacobian of measurement function
Matrix H_enc(const Vector &x)
{
	float theta = x[2];
	float s = sin(theta);
	float c = cos(theta);

	Matrix H(2, 9);

	H(0, 3) = c;					// v_fwd/∂v_x
	H(0, 4) = -s;					// v_fwd/∂v_y
	H(0, 2) = -x[3] * s - x[4] * c; // v_fwd/∂θ

	H(1, 5) = 1; // ∂ω/∂ω

	return H;
}

// Measurement noise covariance
Matrix R_enc = Matrix::Diagonal({1.0e-4f, 2.5e-5f});

ExtendedKalmanFilter ekf;
State robot;

void StartFusionTask(void *argument)
{
	// Create the Extended Kalman Filter
	ekf = ExtendedKalmanFilter(f, F, Q);

	// Set the initial state
	Vector x(9);
	Matrix P = Matrix::Identity(9) * 0.01f;
	ekf.initialize(x, P);

	uint64_t last_time = HAL_GetTick();

	while (true)
	{
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Predict the state
		ekf.predict(Vector{delta_time});

		// Update the state from the state vector
		Vector ekfState = ekf.getState();

		robot.position = ekfState.getSubVector(0, 1);
		robot.orientation = ekfState[2];
		robot.velocity = ekfState.getSubVector(3, 4);
		robot.angular_velocity = ekfState[5];
		robot.acceleration = ekfState.getSubVector(6, 7);
		robot.angular_acceleration = ekfState[8];

		osDelay(10);
	}
}

void StartControlTask(void *argument)
{
	while (true)
	{
		// printf("Hello World!\n");

		osDelay(100);
	}
}
