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

PID turnPID(8.0f, 0.2f, 1.5f, -10.0f, 10.0f, -1.0f, 1.0f);
PID distancePID(5.0f, 0.02f, 1.5f, -1.0f, 1.0f, -0.25f, 0.25f);

Servo servo1(&htim12, TIM_CHANNEL_2, 270.0f, 0.0f);
PWM relay(&htim12, TIM_CHANNEL_1);

extern osMutexId_t uart4MutexHandle;
extern osMutexId_t uart7MutexHandle;

DDSM400 motor1(&huart4, &uart4MutexHandle); // Front left
DDSM400 motor2(&huart7, &uart7MutexHandle); // Front right
DDSM400 motor3(&huart4, &uart4MutexHandle); // Rear left
DDSM400 motor4(&huart7, &uart7MutexHandle); // Rear right

Eigen::Vector2f target{0.0f, 0.0f};
float target_heading = 0.0f;
float previous_target_heading = 0.0f;

// Line following variables
bool returning = false;
Eigen::Vector2f lego_position{0.0f, 0.0f};

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

		// Update the encoders data
		// encoders_data.left.position = motor3.getPosition() * WHEEL_RADIUS;
		encoders_data.left.velocity = motor3.getVelocity() * WHEEL_RADIUS;
		encoders_data.left.data_ready = true;
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

		// Update the encoders data
		// encoders_data.right.position = -motor4.getPosition() * WHEEL_RADIUS;
		encoders_data.right.velocity = -motor4.getVelocity() * WHEEL_RADIUS;
		encoders_data.right.data_ready = true;
		encoders_data.right.active = motor4.getStatus() == DDSM400_FAULT::DDSM400_NONE;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

void drive_to_target(Eigen::Vector2f _target, float _target_heading, uint32_t timeout = UINT32_MAX)
{
	target = _target;
	target_heading = _target_heading;

	float distance_to_target = 0.0f;
	float heading_error = 0.0f;

	uint32_t start_time = osKernelGetTickCount();

	// While we are not within a tolerance of the target heading and position
	do
	{
		Eigen::Vector2f position = robot.position.head<2>();

		// Calculate the distance to the target and heading error
		distance_to_target = (target - position).norm();
		heading_error = normalizeAngle(target_heading - robot.orientation[2]);

		osDelay(10);
	} while ((distance_to_target > 0.03f || fabs(heading_error) > 0.03f) && (osKernelGetTickCount() - start_time) < timeout);
}

void enable()
{
	printf("Enabling the robot...\n");

	// Enable the motors
	left_motor_active = true;
	right_motor_active = true;

	// Set return to false
	returning = false;

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

void pickup(Eigen::Vector2f pickup_target)
{
	printf("Picking up the tape...\n");

	float pickup_distance = 0.155f;

	// float look_heading = normalizeAngle(atan2(pickup_target[1] - robot.position[1], pickup_target[0] - robot.position[0]) - M_PI_2);
	// Eigen::Vector2f target_position = pickup_target - Eigen::Vector2f{pickup_distance * cos(look_heading + M_PI_2), pickup_distance * sin(look_heading + M_PI_2)};

	drive_to_target(robot.position.head<2>(), 85.0f * DEG_TO_RAD, 5000);
	// drive_to_target(target_position, look_heading, 5000);

	servo1.setAngle(SERVO_PICKUP_ANGLE);

	osDelay(750); // Pick up the tape with the servo

	servo1.setAngle(SERVO_DEFAULT_ANGLE);

	drive_to_target(robot.position.head<2>(), -135.0f * DEG_TO_RAD, 5000);

	returning = true;
}

void dropoff()
{
	printf("Dropping off the tape...\n");

	drive_to_target({0.0f, 0.1f}, 180.0f * DEG_TO_RAD, 5000);

	servo1.setAngle(SERVO_DROPOFF_ANGLE);
	relay.on();

	osDelay(2000); // Cut the tape with the relay

	relay.off();
	servo1.setAngle(SERVO_STORAGE_ANGLE);

	disable();
}

void load()
{
	servo1.setAngle(SERVO_PICKUP_ANGLE);
	relay.off();
}

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

	// while (state != RobotMode::ROBOT_MODE_ENABLED)
	// {
	// 	osDelay(10);
	// }

	// target = Eigen::Vector2f{0.0f, 1.0f};
	// target_heading = M_PI_2;

	float initial_time = osKernelGetTickCount() / 1000.0f;
	uint32_t last_time = osKernelGetTickCount();

	uint16_t path_index = 0;

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

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

		// // Set the target over time
		// float t = (osKernelGetTickCount() / 1000.0f) - initial_time;
		// float s = -0.25f; // m/s

		// // Parametric equation of a line
		// target[0] = std::max(-0.3f, s * t);
		// target[1] = std::max(-1.0f, s * t);
		// target_heading = 0.0f;

		// Get the target position
		// Eigen::Vector2f last_target{path[path_index - 1].first, path[path_index - 1].second};
		// Eigen::Vector2f target{path[path_index].first, path[path_index].second};

		// Calculate the euclidean distance to the target
		float distance_to_target = (target - position).norm();
		float angle_to_target = normalizeAngle(robot.orientation[2] - atan2f(target[1] - position[1], target[0] - position[0]) + M_PI_2);

		float target_speed = distancePID.update(distance_to_target, 0, delta_time);

		// If target is behind the robot, reverse
		if (abs(angle_to_target) > M_PI_2)
		{
			target_speed *= -1.0f;
		}

		if (distance_to_target < 0.025f)
		{
			target_speed = 0.0f;
		}

		// Calculate the curvature
		float curvature = PurePursuit<float>::CalculateCurvature(position, robot.orientation[2] + M_PI_2, target);
		float curvature_angular_velocity = curvature * target_speed;

		// Calculate the heading controller
		float heading_error = normalizeAngle(target_heading - robot.orientation[2]);
		float heading_angular_velocity = turnPID.update(heading_error, 0, delta_time);

		// Blend the curvature and heading controllers based on distance to target
		float blend_distance = 0.25f;
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

		if (returning)
		{
			Eigen::Vector2f endpoint{0.0f, 0.25f};
			if ((endpoint - position).norm() < 0.1f && state == RobotMode::ROBOT_MODE_ENABLED)
			{
				commanded_function = RobotFunction::ROBOT_FUNCTION_DROPOFF;
			}
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

	case 0x02: // Cannot see line
	{
		bool last_direction = rx[0] & 0x01; // 1 = right, 0 = left

		if (state == RobotMode::ROBOT_MODE_ENABLED)
		{
			// Delta heading
			target_heading = normalizeAngle(target_heading + copysignf(6.0f * DEG_TO_RAD, last_direction ? 1.0f : -1.0f));
		}
	}
	break;
	case 0x05: // Set robot state
	{
		commanded_function = (RobotFunction)rx[0];

		if (commanded_function == RobotFunction::ROBOT_FUNCTION_PICKUP)
		{
			// Next 4 bytes are the float value in IEEE 754 format
			float dx = 0;
			memcpy(&dx, &rx[0], 4);

			// Next 4 bytes are the float value in IEEE 754 format
			float dy = 0;
			memcpy(&dy, &rx[4], 4);

			float D = 0.1225f;
			float O = 0.018f;

			Eigen::Rotation2D rotation(robot.orientation[2]);

			Eigen::Vector2f robot_position{robot.position[0], robot.position[1]};
			Eigen::Vector2f camera_target_position{dx + O, dy + D};

			lego_position = robot_position + rotation * camera_target_position;
		}
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

		Eigen::Rotation2D rotation(robot.orientation[2]);

		Eigen::Vector2f robot_position{robot.position[0], robot.position[1]};
		Eigen::Vector2f camera_target_position{dx + O, dy + D};

		target = robot_position + rotation * camera_target_position;

		previous_target_heading = target_heading;
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
				pickup(lego_position);
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
				state = RobotMode::ROBOT_MODE_DISABLED;
			}
			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		case RobotFunction::ROBOT_FUNCTION_LOAD:
		{
			// Load the tape
			load();

			commanded_function = RobotFunction::ROBOT_FUNCTION_NONE;
		}
		break;
		}

		osDelayUntil(last_time + 10); // 100 Hz
	}
}