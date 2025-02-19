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

// Sensors
Sensor *sensors[SENSOR_COUNT] = {&accelerometer, &gyroscope, &barometer};

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
		EKF::ControlVector u{delta_time};

		// Predict the state
		ekf.predict(u);

		// Update the state from the state vector
		x = ekf.getState();

		// Check all sensors for updates
		for (int i = 0; i < SENSOR_COUNT; i++)
		{
			Sensor *s = sensors[i];
			if (s->ready())
			{
				// Update the measurement functions and covariance
				ekf.setMeasurement(s->h, s->H, s->R);

				// Get the measurement vector
				EKF::MeasurementVector z = s->z(x);

				// Update the state estimate
				ekf.update(z);
			}
		}

		// Get the state vector
		x = ekf.getState();

		// Update the robot state
		robot.position = Eigen::Vector3f{x[0], x[1], x[2]};
		robot.velocity = Eigen::Vector3f{x[6], x[7], x[8]};
		robot.acceleration = Eigen::Vector3f{x[12], x[13], x[14]};

		robot.orientation = Eigen::Vector3f{x[3], x[4], x[5]};
		robot.angular_velocity = Eigen::Vector3f{x[9], x[10], x[11]};

		osDelay(4); // 250 Hz
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
