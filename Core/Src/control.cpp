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
Sensor *sensors[SENSOR_COUNT] = {&gyroscope};

EKF ekf(f, F, Q);
RobotState robot;

void StartFusionTask(void *argument)
{
	// Set the initial state
	EKF::StateVector x = EKF::StateVector::Zero();
	EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.01f;
	ekf.initialize(x, P);

	osDelay(2000); // Wait for the sensors to initialize

	uint32_t last_time = osKernelGetTickCount();
	osDelay(10); // 100 Hz

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

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

		osDelayUntil(last_time + 4); // 250 Hz
	}
}

PID turnPID(0.5f, 0.01f, 0.0f, -0.2f, 0.2f, -0.2f, 0.2f);

Servo servo1(&htim12, TIM_CHANNEL_2, 270, 0);

extern osMutexId_t uart4MutexHandle;
extern osMutexId_t uart7MutexHandle;

DDSM400 motor1(&huart4, &uart4MutexHandle); // Front left
DDSM400 motor2(&huart7, &uart7MutexHandle); // Front right
DDSM400 motor3(&huart4, &uart4MutexHandle); // Rear left
DDSM400 motor4(&huart7, &uart7MutexHandle); // Rear right

volatile float left_speed = 0;
volatile float right_speed = 0;

void LeftMotorTask(void *argument)
{
	// Motor Initialization
	motor1.init(0x01);
	motor3.init(0x03);

	// Enable the motors
	motor1.enable();
	motor3.enable();

	// Set the default acceleration
	float rotational_acceleration = MAX_ACCELERATION / WHEEL_RADIUS;

	motor1.setDefaultAcceleration(rotational_acceleration);
	motor3.setDefaultAcceleration(rotational_acceleration);

	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		float left_rotational_velocity = left_speed / WHEEL_RADIUS;

		motor1.setVelocity(left_rotational_velocity);
		motor3.setVelocity(left_rotational_velocity);

		float left_encoder_position = motor1.getPosition() * WHEEL_RADIUS;
		float left_encoder_speed = motor1.getVelocity() * WHEEL_RADIUS;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

void RightMotorTask(void *argument)
{
	// Motor Initialization
	motor2.init(0x02);
	motor4.init(0x04);

	// Enable the motors
	motor2.enable();
	motor4.enable();

	// Set the default acceleration
	float rotational_acceleration = MAX_ACCELERATION / WHEEL_RADIUS;

	motor2.setDefaultAcceleration(rotational_acceleration);
	motor4.setDefaultAcceleration(rotational_acceleration);

	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		float right_rotational_velocity = right_speed / WHEEL_RADIUS;

		motor2.setVelocity(-right_rotational_velocity);
		motor4.setVelocity(-right_rotational_velocity);

		float right_encoder_position = motor2.getPosition() * WHEEL_RADIUS;
		float right_encoder_speed = motor2.getVelocity() * WHEEL_RADIUS;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

void StartControlTask(void *argument)
{
	// Motor Tasks
	xTaskCreate(LeftMotorTask, "LeftMotorTask", 256, NULL, 2, NULL);
	xTaskCreate(RightMotorTask, "RightMotorTask", 256, NULL, 2, NULL);

	// // Servo Initialization
	// servo1.init();

	// osDelay(10000);

	// servo1.setAngle(228);

	// osDelay(2000);

	// servo1.setAngle(45);

	// osDelay(2000);

	// servo1.setAngle(230);

	// osDelay(1000);

	// Gyro calibration takes 5 seconds, wait for it to finish
	osDelay(6000);

	// Drive forward for 1 second
	left_speed = 0.25f;
	right_speed = 0.25f;

	osDelay(1000);

	// Stop for 1 second
	left_speed = 0;
	right_speed = 0;

	osDelay(1000);

	// Drive backward for 1 second

	left_speed = -0.25f;
	right_speed = -0.25f;

	osDelay(1000);

	// Stop for 1 second

	left_speed = 0;
	right_speed = 0;

	float initial_time = osKernelGetTickCount() / 1000.0f;
	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		// angle += velocity;

		// if (angle >= max_angle || angle <= min_angle)
		// {
		// 	velocity *= -1.0f;
		// }

		// setServoAngle(SERVO_1, angle);
		// setServoAngle(SERVO_2, max_angle - angle);

		// setServoAngle(SERVO_1, 0);
		// setServoAngle(SERVO_2, 0);

		// Eigen::Vector2f position = robot.position.head<2>();
		// Eigen::Vector2f target{0, 0}; // GetTarget();

		// // Calculate the curvature
		// float curvature = PurePursuit<float>::CalculateCurvature(position, robot.orientation[2], target);

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

		// float time = (osKernelGetTickCount() / 1000.0f) - initial_time;

		// float speed = 0.5f * sinf(0.2 * M_PI * time);

		// left_speed = speed;
		// right_speed = speed;

		float speed = turnPID.update(M_PI_2, robot.orientation[0], delta_time);

		left_speed = speed;
		right_speed = -speed;

		// CSV print
		// printf("%.2f, %.4f, %.4f\n", time, left_position, left_speed);

		osDelayUntil(last_time + 20); // 50 Hz
	}
}

void StartCommTask(void *argument)
{
	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		// printf("Hello, world!\n");

		osDelayUntil(last_time + 1000); // 1 Hz
	}
}