#include "control.h"

/**
 * Normalize angle to range [-pi, pi]
 */
float normalizeAngle(float angle)
{
	while (abs(angle) > M_PI)
	{
		angle -= copysignf(2 * M_PI, angle);
	}
	return angle;
}

// State transition function
EKF::ProcessVector f(const EKF::StateVector &x, const EKF::ControlVector &u)
{
	float dt = u[0];
	float dt2 = 0.5f * dt * dt;

	float phi = x[3] + x[9] * dt;
	float theta = x[4] + x[10] * dt;
	float psi = x[5] + x[11] * dt;

	return EKF::StateVector{
		x[0] + x[6] * dt + x[12] * dt2, // x = x + x' * Δt + 0.5 * x'' * Δt^2
		x[1] + x[7] * dt + x[13] * dt2, // y = y + y' * Δt + 0.5 * y'' * Δt^2
		x[2] + x[8] * dt + x[14] * dt2, // z = z + z' * Δt + 0.5 * z'' * Δt^2
		normalizeAngle(phi),			// φ = φ + φ' * Δt
		normalizeAngle(theta),			// θ = θ + θ' * Δt
		normalizeAngle(psi),			// ψ = ψ + ψ' * Δt
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

// 90 degrees on the z-axis
Eigen::Vector3f mounting_orientation{
	0.0f,			   // φ (roll)
	0.0f,			   // θ (pitch)
	DEG_TO_RAD * 90.0f // ψ (yaw)
};

// Sensors
Sensor *sensors[SENSOR_COUNT] = {&gyroscope, &encoders};

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

		// Update the state from the state vector for the sensors
		x = ekf.getState();

		// // Get orientation from the state vector x
		// Eigen::Vector3f orientation{
		// 	x[3], // φ (roll)
		// 	x[4], // θ (pitch)
		// 	x[5]  // ψ (yaw)
		// };

		// // Construct rotation matrices using the ZYX (yaw-pitch-roll) convention.
		// Eigen::Matrix3f R_mounting = eulerToMatrix(mounting_orientation);

		// // To undo the mounting offset, apply the inverse of R_mounting.
		// Eigen::Vector3f offset_orientation = R_mounting.transpose() * orientation;

		// // Update the state vector
		// state[3] = offset_orientation[0];
		// state[4] = offset_orientation[1];
		// state[5] = offset_orientation[2];

		// Check all sensors for updates
		for (uint8_t i = 0; i < SENSOR_COUNT; i++)
		{
			Sensor *s = sensors[i];
			if (s->ready())
			{
				// Update the measurement functions and covariance
				ekf.setMeasurement(s->h, s->H, s->R);

				// Get the measurement vector
				EKF::MeasurementVector z = s->z(x);

				// Mark the sensor as not ready
				s->consume();

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

		osDelayUntil(last_time + 10); // 100 Hz
	}
}

RobotMode state = RobotMode::ROBOT_MODE_DISABLED;
RobotFunction commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;

PID turnPID(9.0f, 0.0f, 0.0f, -5.0f, 5.0f);
PID distancePID(1.2f, 0.02f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f);

Servo servo1(&htim12, TIM_CHANNEL_2, 270, 0);
PWM relay(&htim12, TIM_CHANNEL_1);

extern osMutexId_t uart4MutexHandle;
extern osMutexId_t uart7MutexHandle;

DDSM400 motor1(&huart4, &uart4MutexHandle); // Front left
DDSM400 motor2(&huart7, &uart7MutexHandle); // Front right
DDSM400 motor3(&huart4, &uart4MutexHandle); // Rear left
DDSM400 motor4(&huart7, &uart7MutexHandle); // Rear right

volatile float left_velocity = 0;
bool left_motor_active = false;

void StartLeftMotorTask(void *argument)
{
	// Motor Initialization
	motor1.init(0x01, false, true);
	motor3.init(0x03);

	// Disable the motors
	motor1.disable();
	motor3.disable();

	// Set the default acceleration
	float rotational_acceleration = MAX_ACCELERATION / WHEEL_RADIUS;

	motor1.setDefaultAcceleration(rotational_acceleration);
	motor3.setDefaultAcceleration(rotational_acceleration);

	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		if (left_motor_active)
		{
			if (motor1.getMode() == DDSM400_MODE::DDSM400_DISABLED)
			{
				motor1.enable();
			}

			if (motor3.getMode() == DDSM400_MODE::DDSM400_DISABLED)
			{
				motor3.enable();
			}

			float left_rotational_velocity = left_velocity / WHEEL_RADIUS;

			// Set the motor velocity
			motor1.setVelocity(left_rotational_velocity);
			motor3.setVelocity(left_rotational_velocity);

			// Update the encoders data
			encoders_data.left.position = motor3.getPosition() * WHEEL_RADIUS;
			encoders_data.left.velocity = motor3.getVelocity() * WHEEL_RADIUS;
			encoders_data.left.data_ready = true;
		}
		else
		{
			left_velocity = 0;

			if (motor1.getMode() != DDSM400_MODE::DDSM400_DISABLED)
			{
				motor1.disable();
			}

			if (motor3.getMode() != DDSM400_MODE::DDSM400_DISABLED)
			{
				motor3.disable();
			}
		}

		encoders_data.left.active = motor3.getStatus() == DDSM400_FAULT::DDSM400_NONE;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

volatile float right_velocity = 0;
bool right_motor_active = false;

void StartRightMotorTask(void *argument)
{
	// Motor Initialization
	motor2.init(0x02, false, true);
	motor4.init(0x04);

	// Disable the motors
	motor2.disable();
	motor4.disable();

	// Set the default acceleration
	float rotational_acceleration = MAX_ACCELERATION / WHEEL_RADIUS;

	motor2.setDefaultAcceleration(rotational_acceleration);
	motor4.setDefaultAcceleration(rotational_acceleration);

	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		if (right_motor_active)
		{
			if (motor2.getMode() == DDSM400_MODE::DDSM400_DISABLED)
			{
				motor2.enable();
			}

			if (motor4.getMode() == DDSM400_MODE::DDSM400_DISABLED)
			{
				motor4.enable();
			}

			float right_rotational_velocity = -right_velocity / WHEEL_RADIUS;

			// Set the motor velocity
			motor2.setVelocity(right_rotational_velocity);
			motor4.setVelocity(right_rotational_velocity);

			// Update the encoders data
			encoders_data.right.position = -motor4.getPosition() * WHEEL_RADIUS;
			encoders_data.right.velocity = -motor4.getVelocity() * WHEEL_RADIUS;
			encoders_data.right.data_ready = true;
		}
		else
		{
			right_velocity = 0;

			if (motor2.getMode() != DDSM400_MODE::DDSM400_DISABLED)
			{
				motor2.disable();
			}

			if (motor4.getMode() != DDSM400_MODE::DDSM400_DISABLED)
			{
				motor4.disable();
			}
		}

		encoders_data.right.active = motor4.getStatus() == DDSM400_FAULT::DDSM400_NONE;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

void pickup()
{
	printf("Picking up the tape...\n");

	servo1.setAngle(SERVO_PICKUP_ANGLE);

	osDelay(750); // Pick up the tape with the servo

	servo1.setAngle(SERVO_DEFAULT_ANGLE);
}

void dropoff()
{
	printf("Dropping off the tape...\n");

	servo1.setAngle(SERVO_DROPOFF_ANGLE);
	relay.on();

	osDelay(1500); // Cut the tape with the relay

	relay.off();
	servo1.setAngle(SERVO_DEFAULT_ANGLE);
}

void enable()
{
	printf("Enabling the robot...\n");

	// Reset the robot state
	ekf.reset();

	// Enable the motors
	left_motor_active = true;
	right_motor_active = true;

	// Set servo to initial position
	servo1.setAngle(SERVO_DEFAULT_ANGLE);

	RUN(setBuzzer, 750, MEDIUM_POWER * SLOW_BLINK, OFF_POWER);
}

void disable()
{
	printf("Disabling the robot...\n");

	// Disable the motors
	left_motor_active = false;
	right_motor_active = false;

	// Reset the servo and relay
	servo1.setAngle(SERVO_STORAGE_ANGLE);
	relay.off();

	RUN(setBuzzer, 1550, MEDIUM_POWER * BLINK_2, OFF_POWER);
}

#include <vector>

std::vector<std::pair<float, float>> path = {
	{0, 0},
	{0, 0.025},
	{0, 0.05},
	{0, 0.075},
	{0, 0.1},
	{0, 0.125},
	{0, 0.15},
	{0, 0.175},
	{0, 0.2},
	{0, 0.225},
	{0, 0.25},
	{0, 0.275},
	{0, 0.3},
	{0, 0.325},
	{0, 0.35},
	{0, 0.375},
	{0, 0.4},
	{0, 0.425},
	{0, 0.45},
	{0, 0.475},
	{0, 0.5},
	{0, 0.525},
	{0, 0.55},
	{0, 0.575},
	{0, 0.6},
	{0, 0.625},
	{0, 0.65},
	{0, 0.675},
	{0, 0.7},
	{0, 0.725},
	{0, 0.75},
	{0, 0.775},
	{0, 0.8},
	{0, 0.825},
	{0, 0.85},
	{0, 0.875},
	{0, 0.9},
	{0, 0.925},
	{0, 0.95},
	{0, 0.975},
	{0, 1},
	{0, 1.025},
	{0, 1.05},
	{0, 1.075},
	{0, 1.1},
	{0, 1.125},
	{0, 1.15},
	{0, 1.175},
	{0, 1.2},
	{0, 1.225},
	{0, 1.25},
	{0, 1.275},
	{0, 1.3},
	{0, 1.325},
	{0, 1.35},
	{0.0018, 1.3735},
	{0.0073, 1.3964},
	{0.0163, 1.4181},
	{0.0286, 1.4382},
	{0.0439, 1.4561},
	{0.0618, 1.4714},
	{0.0819, 1.4837},
	{0.1036, 1.4927},
	{0.1265, 1.4982},
	{0.15, 1.5},
	{0.175, 1.5},
	{0.2, 1.5},
	{0.225, 1.5},
	{0.25, 1.5},
	{0.275, 1.5},
	{0.3, 1.5},
	{0.325, 1.5},
	{0.35, 1.5},
	{0.375, 1.5},
	{0.4, 1.5},
	{0.425, 1.5},
	{0.45, 1.5},
	{0.475, 1.5},
	{0.5, 1.5},
	{0.525, 1.5},
	{0.55, 1.5},
	{0.575, 1.5},
	{0.6, 1.5},
	{0.625, 1.5},
	{0.65, 1.5},
	{0.675, 1.5},
	{0.7, 1.5},
	{0.725, 1.5},
	{0.75, 1.5},
	{0.775, 1.5},
	{0.8, 1.5},
	{0.825, 1.5},
	{0.85, 1.5},
	{0.875, 1.5},
	{0.9, 1.5},
	{0.925, 1.5},
	{0.95, 1.5},
	{0.975, 1.5},
	{1, 1.5},
	{1.025, 1.5},
	{1.05, 1.5},
	{1.075, 1.5},
	{1.1, 1.5},
	{1.125, 1.5},
	{1.15, 1.5},
	{1.175, 1.5},
	{1.2, 1.5},
	{1.225, 1.5},
	{1.25, 1.5},
	{1.275, 1.5},
	{1.3, 1.5},
	{1.325, 1.5},
	{1.35, 1.5},
	{1.3735, 1.4982},
	{1.3964, 1.4927},
	{1.4181, 1.4837},
	{1.4382, 1.4714},
	{1.4561, 1.4561},
	{1.4714, 1.4382},
	{1.4837, 1.4181},
	{1.4927, 1.3964},
	{1.4982, 1.3735},
	{1.5, 1.35},
	{1.4982, 1.3265},
	{1.4927, 1.3036},
	{1.4837, 1.2819},
	{1.4714, 1.2618},
	{1.4561, 1.2439},
	{1.4382, 1.2286},
	{1.4181, 1.2163},
	{1.3964, 1.2073},
	{1.3735, 1.2018},
	{1.35, 1.2},
	{1.3265, 1.1982},
	{1.3036, 1.1927},
	{1.2819, 1.1837},
	{1.2618, 1.1714},
	{1.2439, 1.1561},
	{1.2286, 1.1382},
	{1.2163, 1.1181},
	{1.2073, 1.0964},
	{1.2018, 1.0735},
	{1.2, 1.05},
	{1.2018, 1.0265},
	{1.2073, 1.0036},
	{1.2163, 0.9819},
	{1.2286, 0.9618},
	{1.2439, 0.9439},
	{1.2618, 0.9286},
	{1.2819, 0.9163},
	{1.3036, 0.9073},
	{1.3265, 0.9018},
	{1.35, 0.9},
	{1.3735, 0.8982},
	{1.3964, 0.8927},
	{1.4181, 0.8837},
	{1.4382, 0.8714},
	{1.4561, 0.8561},
	{1.4714, 0.8382},
	{1.4837, 0.8181},
	{1.4927, 0.7964},
	{1.4982, 0.7735},
	{1.5, 0.75},
	{1.5, 0.725},
	{1.5, 0.7},
	{1.5, 0.675},
	{1.5, 0.65},
	{1.5, 0.625},
	{1.5, 0.6},
	{1.5, 0.575},
	{1.5, 0.55},
	{1.5, 0.525},
	{1.5, 0.5},
	{1.5, 0.475},
	{1.5, 0.45},
	{1.5, 0.425},
	{1.5, 0.4},
	{1.5, 0.375},
	{1.5, 0.35},
	{1.5, 0.325},
	{1.5, 0.3},
	{1.5, 0.275},
	{1.5, 0.25},
	{1.5, 0.225},
	{1.5, 0.2},
	{1.5, 0.175},
	{1.5, 0.15},
	{1.4982, 0.1265},
	{1.4927, 0.1036},
	{1.4837, 0.0819},
	{1.4714, 0.0618},
	{1.4561, 0.0439},
	{1.4382, 0.0286},
	{1.4181, 0.0163},
	{1.3964, 0.0073},
	{1.3735, 0.0018},
	{1.35, 0},
	{1.325, 0},
	{1.3, 0},
	{1.275, 0},
	{1.25, 0},
	{1.225, 0},
	{1.2, 0},
	{1.175, 0},
	{1.15, 0},
	{1.125, 0},
	{1.1, 0},
	{1.075, 0},
	{1.05, 0},
	{1.0265, 0.0018},
	{1.0036, 0.0073},
	{0.9819, 0.0163},
	{0.9618, 0.0286},
	{0.9439, 0.0439},
	{0.9286, 0.0618},
	{0.9163, 0.0819},
	{0.9073, 0.1036},
	{0.9018, 0.1265},
	{0.9, 0.15},
	{0.9, 0.175},
	{0.9, 0.2},
	{0.9, 0.225},
	{0.9, 0.25},
	{0.9, 0.275},
	{0.9, 0.3},
	{0.9, 0.325},
	{0.9, 0.35},
	{0.9, 0.375},
	{0.9, 0.4},
	{0.9, 0.425},
	{0.9, 0.45},
	{0.8982, 0.4735},
	{0.8927, 0.4964},
	{0.8837, 0.5181},
	{0.8714, 0.5382},
	{0.8561, 0.5561},
	{0.8382, 0.5714},
	{0.8181, 0.5837},
	{0.7964, 0.5927},
	{0.7735, 0.5982},
	{0.75, 0.6},
	{0.725, 0.6},
	{0.7, 0.6},
	{0.675, 0.6},
	{0.65, 0.6},
	{0.625, 0.6},
	{0.6, 0.6},
};

Eigen::Vector2f target{0.0f, 0.0f};
float target_heading = 0.0f;

void StartControlTask(void *argument)
{
	// Servo Initialization
	servo1.init(SERVO_STORAGE_ANGLE);

	// Relay Initialization
	relay.init();
	relay.off();

	// Gyro calibration takes 5 seconds, wait for it to finish
	osDelay(5000);

	// Enable the robot
	// commanded_function = RobotFunction::ROBOT_FUNCTION_ENABLE;

	float initial_time = osKernelGetTickCount() / 1000.0f;
	uint32_t last_time = osKernelGetTickCount();

	uint16_t path_index = 0;

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		// Update the robot state
		switch (commanded_function)
		{
		case RobotFunction::ROBOT_FUNCTION_ENABLE:
		{
			if (state == RobotMode::ROBOT_MODE_DISABLED) // If the robot is disabled
			{
				state = RobotMode::ROBOT_MODE_ENABLING_TRANSITION;
				enable();
				state = RobotMode::ROBOT_MODE_ENABLED;
			}
			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		case RobotFunction::ROBOT_FUNCTION_DISABLE:
		{
			if (state != RobotMode::ROBOT_MODE_DISABLED) // If the robot is not disabled
			{
				state = RobotMode::ROBOT_MODE_DISABLING_TRANSITION;
				disable();
				state = RobotMode::ROBOT_MODE_DISABLED;
			}
			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		case RobotFunction::ROBOT_FUNCTION_PICKUP:
		{
			if (state == RobotMode::ROBOT_MODE_ENABLED) // If the robot is enabled
			{
				state = RobotMode::ROBOT_MODE_PICKUP_TRANSITION;
				pickup();
				state = RobotMode::ROBOT_MODE_ENABLED;
			}
			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		case RobotFunction::ROBOT_FUNCTION_DROPOFF:
		{
			if (state == RobotMode::ROBOT_MODE_ENABLED) // If the robot is enabled
			{
				state = RobotMode::ROBOT_MODE_DROPOFF_TRANSITION;
				dropoff();
				state = RobotMode::ROBOT_MODE_ENABLED;
			}
			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		}

		if (state == RobotMode::ROBOT_MODE_DISABLED)
		{
			// Stop the robot
			left_velocity = 0.0f;
			right_velocity = 0.0f;

			// Reset the initial time
			initial_time = osKernelGetTickCount() / 1000.0f;

			osDelayUntil(last_time + 10); // 100 Hz
			continue;
		}

		Eigen::Vector2f position = robot.position.head<2>();

		// Get the target position
		// Eigen::Vector2f last_target{path[path_index - 1].first, path[path_index - 1].second};
		// Eigen::Vector2f target{path[path_index].first, path[path_index].second};

		// Calculate the euclidean distance to the target
		float distance_to_target = (target - position).norm();
		float angle_to_target = atan2f(target[1] - position[1], target[0] - position[0]);

		float target_speed = distancePID.update(distance_to_target, 0, delta_time);

		// If target is behind the robot, reverse
		// if (angle_to_target < 0)
		// {
		// 	target_speed *= -1;
		// }

		// Calculate the curvature
		float curvature = PurePursuit<float>::CalculateCurvature(position, robot.orientation[2] + M_PI_2, target);
		float curvature_angular_velocity = curvature * target_speed;

		// Calculate the heading controller
		float heading_error = normalizeAngle(target_heading - robot.orientation[2]);
		float heading_angular_velocity = turnPID.update(heading_error, 0, delta_time);

		// Blend the curvature and heading controllers based on distance to target
		float blend_distance = 0.18f;
		float blend = std::min(distance_to_target / blend_distance, 1.0f); // when blend = 1 -> curvature_speed, when blend = 0 -> heading_speed
		float angular_velocity = (1 - blend) * heading_angular_velocity + blend * curvature_angular_velocity;

		// Calculate the left and right wheel velocities
		left_velocity = target_speed - angular_velocity * WHEEL_DISTANCE / 2.0f;
		right_velocity = target_speed + angular_velocity * WHEEL_DISTANCE / 2.0f;

		// Limit the wheel velocities to the maximum speed
		float max_velocity = std::max(abs(left_velocity), abs(right_velocity));

		// If the maximum velocity is greater than the maximum speed (maintain the ratio)
		if (max_velocity > MAX_SPEED)
		{
			left_velocity *= MAX_SPEED / max_velocity;
			right_velocity *= MAX_SPEED / max_velocity;
		}

		// If the robot is within 3 cm of the target stop
		if (distance_to_target < 0.03f)
		{
			left_velocity = 0.0f;
			right_velocity = 0.0f;
		}

		// If the robot is close to the target, move to the next target
		// float distance_threshold = 0.04f;
		// if (distance_to_target < distance_threshold)
		// {
		// 	if (path_index == path.size() - 1)
		// 	{
		// 		// Stop the robot
		// 		left_velocity = 0.0f;
		// 		right_velocity = 0.0f;
		// 	}
		// 	else
		// 	{
		// 		path_index = (path_index + 1) % path.size();
		// 	}
		// }

		osDelayUntil(last_time + 10); // 100 Hz
	}
}

// I2C Communication
uint8_t _rx_count = 0;
uint8_t _rx[256] = {0};

uint8_t _tx_count = 0;
uint8_t _tx[256] = {0};

void Process_RX_Data(uint8_t command, uint8_t *rx, uint8_t rx_count)
{
	// printf("Command: 0x%02X\n", command);

	// Print the received data
	printf("Received: [ ");
	for (uint8_t i = 0; i < rx_count + 2; i++)
	{
		printf("0x%02X ", rx[i - 2]);
	}
	printf("]\n");

	switch (command)
	{
		// TRANSMITTING DATA

	case 0x05: // Set robot state
	{
		commanded_function = (RobotFunction)rx[0];
	}
	break;
	case 0x10: // New target position
	{
		// Next 4 bytes are the float value in IEEE 754 format
		float dx = 0;
		memcpy(&dx, &rx[0], 4);

		// Next 4 bytes are the float value in IEEE 754 format
		float dy = 0;
		memcpy(&dy, &rx[4], 4);

		// Next 4 bytes are the float value in IEEE 754 format
		float theta = 0;
		memcpy(&theta, &rx[8], 4);

		float D = 0.1225f;
		float O = 0.018f;

		float phi = robot.orientation[2];

		Eigen::Rotation2D rotation(phi);

		Eigen::Vector2f robot_position{robot.position[0], robot.position[1]};
		Eigen::Vector2f camera_target_position{dx + O, dy + D};

		target = robot_position + rotation * camera_target_position;

		target_heading = normalizeAngle(theta + robot.orientation[2] - M_PI_2);

		// printf("Command: %.4f %.4f %.4f\n", target_position[0], target_position[1], theta);
	}
	break;

		// REQUESTING DATA

	case 0x81: // Request robot position
	{
		printf("Requesting robot position\n");

		// X position
		float x = robot.position[0];
		memcpy(&_tx[0], &x, 4);

		// Y position
		float y = robot.position[1];
		memcpy(&_tx[4], &y, 4);

		// Z position
		float t = robot.orientation[2];
		memcpy(&_tx[8], &t, 4);
	}
	break;
	case 0x82: // Request robot velocity
	{
		printf("Requesting robot velocity\n");

		// X velocity
		float x = robot.velocity[0];
		memcpy(&_tx[0], &x, 4);

		// Y velocity
		float y = robot.velocity[1];
		memcpy(&_tx[4], &y, 4);

		// Z velocity
		float t = robot.angular_velocity[2];
		memcpy(&_tx[8], &t, 4);
	}
	break;
	case 0x83: // Request robot acceleration
	{
		printf("Requesting robot acceleration\n");

		// X acceleration
		float x = robot.acceleration[0];
		memcpy(&_tx[0], &x, 4);

		// Y acceleration
		float y = robot.acceleration[1];
		memcpy(&_tx[4], &y, 4);

		// Z acceleration
		float t = robot.acceleration[2];
		memcpy(&_tx[8], &t, 4);
	}
	break;
	case 0x85: // Request robot state
	{
		printf("Requesting robot state\n");

		_tx[0] = state;
	}
	break;
	case 0x90: // Request target point
	{
		printf("Requesting target point\n");

		// memcpy(&_tx[0], &target[0], 4);
		// memcpy(&_tx[4], &target[1], 4);
	}
	default:
		break;
	}
}

void I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // if the master wants to transmit the data
	{
		printf("Transmitting data\n");
		_rx_count = 0;

		// Receive using sequential function.
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, _rx + _rx_count, 1, I2C_FIRST_FRAME);
	}
	else if (TransferDirection == I2C_DIRECTION_RECEIVE) // if the master wants to receive the data
	{
		printf("Receiving data\n");
		_tx_count = 0;

		// Transmit using sequential function.
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, _tx + _tx_count, 1, I2C_FIRST_FRAME);
	}
	else
	{
		Error_Handler();
	}
}

void I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("Received: 0x%02X\n", _rx[_rx_count]);

	uint8_t read = _rx[0] >> 7;
	uint8_t rx_count_max = _rx[1] + 1;

	if (!read && (_rx_count == 0 || _rx_count < rx_count_max)) // If there are more bytes to receive
	{
		if (_rx_count == rx_count_max - 1)
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, _rx + _rx_count + 1, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, _rx + _rx_count + 1, 1, I2C_NEXT_FRAME);
		}
	}
	else // If all the bytes have been received
	{
		Process_RX_Data(_rx[0], _rx + 2, _rx[1]);
	}

	_rx_count++;
}

void I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, _tx + _tx_count, 1, I2C_NEXT_FRAME);

	_tx_count++;
}

void StartCommTask(void *argument)
{
	uint32_t last_time = osKernelGetTickCount();

	hi2c1.ListenCpltCallback = I2C_ListenCpltCallback;
	hi2c1.AddrCallback = I2C_AddrCallback;
	hi2c1.SlaveRxCpltCallback = I2C_SlaveRxCpltCallback;
	hi2c1.SlaveTxCpltCallback = I2C_SlaveTxCpltCallback;

	HAL_I2C_EnableListen_IT(&hi2c1);

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		osDelayUntil(last_time + 1000); // 1 Hz
	}
}