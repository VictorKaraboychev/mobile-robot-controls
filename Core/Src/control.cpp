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
Matrix Q = Matrix{
	{0.01f, 0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0.01f, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0.01f, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0.01f, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0.01f, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0.01f, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0.01f, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0.01f, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0.01f}};

// ---IMU (accelerometer and gyroscope)---

// Measurement function
Vector h_imu(const Vector &x)
{
	float theta = x[2];

	return Vector{
		x[6] * cos(theta) - x[7] * sin(theta), // a_x_body = a_x * cos(θ) - a_y * sin(θ)
		x[6] * sin(theta) + x[7] * cos(theta), // a_y_body = a_x * sin(θ) + a_y * cos(θ)
		x[5]								   // ω
	};
}

// Jacobian of measurement function
Matrix H_imu(const Vector &x)
{
	float theta = x[2];

	Matrix H(3, 9);

	H(0, 6) = cos(theta);							  // ∂a_x_body/∂a_x
	H(0, 7) = -sin(theta);							  // ∂a_x_body/∂a_y
	H(0, 2) = -x[6] * sin(theta) - x[7] * cos(theta); // ∂a_x_body/∂θ

	H(1, 6) = sin(theta);							 // ∂a_y_body/∂a_x
	H(1, 7) = cos(theta);							 // ∂a_y_body/∂a_y
	H(1, 2) = x[6] * cos(theta) - x[7] * sin(theta); // ∂a_y_body/∂θ

	H(2, 5) = 1; // ∂ω/∂w

	return H;
}

// Measurement noise covariance
Matrix R_imu = Matrix{
	{0.01f, 0, 0},
	{0, 0.01f, 0},
	{0, 0, 0.01f}};

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
Matrix R_mag = Matrix{
	{0.01f}};

// ---Encoders---

// Measurement function
Vector h_enc(const Vector &x)
{
	float theta = x[2];

	return Vector{
		x[3] * cos(theta) - x[4] * sin(theta), // v_fwd = v_x * cos(θ) - v_y * sin(θ)
		x[5]								   // ω = ω
	};
}

// Jacobian of measurement function
Matrix H_enc(const Vector &x)
{
	float theta = x[2];

	Matrix H(2, 9);

	H(0, 3) = cos(theta);							  // v_fwd/∂v_x
	H(0, 4) = -sin(theta);							  // v_fwd/∂v_y
	H(0, 2) = -x[3] * sin(theta) - x[4] * cos(theta); // v_fwd/∂θ

	H(1, 5) = 1; // ∂ω/∂ω

	return H;
}

// Measurement noise covariance
Matrix R_enc = Matrix{
	{0.01f, 0},
	{0, 0.01f}};

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
		state.position = ekf.getState().getSubVector(0, 1);
		state.orientation = ekf.getState()[2];
		state.velocity = ekf.getState().getSubVector(3, 4);
		state.angular_velocity = ekf.getState()[5];
		state.acceleration = ekf.getState().getSubVector(6, 7);
		state.angular_acceleration = ekf.getState()[8];	

		osDelay(10);
	}
}