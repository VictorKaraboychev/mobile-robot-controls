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

// PID turnPID(0.5f, 0.01f, 0.0f, -0.2f, 0.2f, -0.2f, 0.2f);
PID distancePID(0.75f, 0.01f, 0.0f, -1.0f, 1.0f, -1.0f, 1.0f);

Servo servo1(&htim12, TIM_CHANNEL_2, 270, 0);
PWM relay(&htim12, TIM_CHANNEL_1);

extern osMutexId_t uart4MutexHandle;
extern osMutexId_t uart7MutexHandle;

DDSM400 motor1(&huart4, &uart4MutexHandle); // Front left
DDSM400 motor2(&huart7, &uart7MutexHandle); // Front right
DDSM400 motor3(&huart4, &uart4MutexHandle); // Rear left
DDSM400 motor4(&huart7, &uart7MutexHandle); // Rear right

volatile float left_velocity = 0;
volatile float right_velocity = 0;

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

		float left_rotational_velocity = left_velocity / WHEEL_RADIUS;

		// Set the motor velocity
		motor1.setVelocity(left_rotational_velocity);
		motor3.setVelocity(left_rotational_velocity);

		// Update the encoders data
		encoders_data.left.position = motor3.getPosition() * WHEEL_RADIUS;
		encoders_data.left.velocity = motor3.getVelocity() * WHEEL_RADIUS;
		encoders_data.left.data_ready = true;
		encoders_data.left.active = motor3.getStatus() == DDSM400_FAULT::NONE;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

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

		float right_rotational_velocity = right_velocity / WHEEL_RADIUS;

		// Set the motor velocity
		motor2.setVelocity(-right_rotational_velocity);
		motor4.setVelocity(-right_rotational_velocity);

		// Update the encoders data
		encoders_data.right.position = -motor4.getPosition() * WHEEL_RADIUS;
		encoders_data.right.velocity = -motor4.getVelocity() * WHEEL_RADIUS;
		encoders_data.right.data_ready = true;
		encoders_data.right.active = motor4.getStatus() == DDSM400_FAULT::NONE;

		osDelayUntil(last_time + 25); // 40 Hz
	}
}

void pickup()
{
	servo1.setAngle(228);

	osDelay(750); // Pick up the tape with the servo

	servo1.setAngle(40);
}

void dropoff()
{
	servo1.setAngle(200);
	relay.on();

	osDelay(1500); // Cut the tape with the relay

	relay.off();
	servo1.setAngle(40);
}

void enable()
{
	// Enable the motors
	motor1.enable();
	motor2.enable();
	motor3.enable();
	motor4.enable();
}

void disable()
{
	// Disable the motors
	motor1.disable();
	motor2.disable();
	motor3.disable();
	motor4.disable();

	// Reset the servo and relay
	servo1.setAngle(40);
	relay.off();

	float t1 = osKernelGetTickCount();

	while (osKernelGetTickCount() - t1 < 1550)
	{
		setBuzzer(MEDIUM_POWER * BLINK_2);
		osDelay(10);
	}
	setBuzzer(OFF_POWER);
}

bool state = false;
Eigen::Vector2f target{0, 0};

void StartControlTask(void *argument)
{
	// Gyro calibration takes 5 seconds, wait for it to finish
	osDelay(6000);

	// Servo Initialization
	servo1.init(40);

	// Relay Initialization
	relay.init();

	bool last_state = false;

	float initial_time = osKernelGetTickCount() / 1000.0f;
	uint32_t last_time = osKernelGetTickCount();

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		if (!state)
		{
			left_velocity = 0.0f;
			right_velocity = 0.0f;

			setBuzzer(0.0f);

			if (last_state) // Transition from enabled to disabled
			{
				disable();

				last_state = false;
			}

			osDelayUntil(last_time + 10); // 100 Hz
			continue;
		}
		else
		{
			if (!last_state) // Transition from disabled to enabled
			{
				initial_time = osKernelGetTickCount() / 1000.0f;

				enable();

				last_state = true;
			}
		}

		Eigen::Vector2f position = robot.position.head<2>();

		// Calculate the euclidean distance to the target
		float distance_to_target = (target - position).norm();
		float angle_to_target = atan2f(target[1] - position[1], target[0] - position[0]);

		float target_speed = distancePID.update(distance_to_target, 0, delta_time);

		// If target is behind the robot, reverse
		if (angle_to_target < 0)
		{
			target_speed *= -1;
		}

		// Calculate the curvature
		float curvature = PurePursuit<float>::CalculateCurvature(position, robot.orientation[2] + M_PI_2, target);

		// Calculate the left and right wheel velocities
		left_velocity = target_speed * (1 - curvature * WHEEL_DISTANCE / 2.0f);
		right_velocity = target_speed * (1 + curvature * WHEEL_DISTANCE / 2.0f);

		// Limit the wheel velocities to the maximum speed
		float max_velocity = std::max(abs(left_velocity), abs(right_velocity));

		// If the maximum velocity is greater than the maximum speed (maintain the ratio)
		if (max_velocity > MAX_SPEED)
		{
			left_velocity *= MAX_SPEED / max_velocity;
			right_velocity *= MAX_SPEED / max_velocity;
		}

		// If the distance is less than the threshold, stop
		float threshold = 0.05f;
		if (distance_to_target < threshold)
		{
			left_velocity = 0.0f;
			right_velocity = 0.0f;
		}

		osDelayUntil(last_time + 10); // 100 Hz
	}
}

// I2C Communication
uint8_t rx_count = 0;
uint8_t rx_count_max = 0;
uint8_t rx[256];

void Process_Data()
{
	uint8_t command = rx[0];

	switch (command)
	{
	case 0x01: // Done the path
	{
	}
	break;
	case 0x02: // No path detected
	{
		target = robot.position.head<2>();
	}
	break;
	case 0x05: // Enabled and ready to start
	{
		state = true;
	}
	break;
	case 0x06: // Disabled
	{
		state = false;
	}
	case 0x10: // New target position
	{
		// Next 4 bytes are the float value in IEEE 754 format
		float dx = 0;
		memcpy(&dx, &rx[1], 4);

		// Next 4 bytes are the float value in IEEE 754 format
		float dy = 0;
		memcpy(&dy, &rx[5], 4);

		// Next 4 bytes are the float value in IEEE 754 format
		float theta = 0;
		memcpy(&theta, &rx[9], 4);

		float R = 0.04f;
		float D = 0.15f;
		float O = 0.0f; //-0.018f;

		float phi = robot.orientation[2];

		Eigen::Rotation2D rotation(phi);

		Eigen::Vector2f robot_position{robot.position[0], robot.position[1]};
		Eigen::Vector2f camera_target_position{dx + O, R * dy + D};

		target = robot_position + rotation * camera_target_position;

		// printf("Command: %.4f %.4f %.4f\n", target_position[0], target_position[1], theta);
	}
	break;
	case 0x11: // Pickup
	{
		pickup();
	}
	break;
	case 0x12: // Dropoff
	{
		dropoff();
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
		rx_count = 0;

		// receive using sequential function.
		HAL_I2C_Slave_Sequential_Receive_IT(hi2c, &rx_count_max, 1, I2C_FIRST_FRAME);
	}
	else if (TransferDirection == I2C_DIRECTION_RECEIVE) // if the master wants to receive the data
	{
		rx_count = 0;
	}
	else
	{
		Error_Handler();
	}
}

void I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rx_count++;

	if (rx_count < rx_count_max) // If there are more bytes to receive
	{
		if (rx_count == rx_count_max - 1)
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rx + rx_count - 1, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, rx + rx_count - 1, 1, I2C_NEXT_FRAME);
		}
	}

	if (rx_count == rx_count_max) // If all the bytes have been received
	{
		Process_Data();
	}
}

void StartCommTask(void *argument)
{
	uint32_t last_time = osKernelGetTickCount();

	hi2c1.ListenCpltCallback = I2C_ListenCpltCallback;
	hi2c1.AddrCallback = I2C_AddrCallback;
	hi2c1.SlaveRxCpltCallback = I2C_SlaveRxCpltCallback;

	HAL_I2C_EnableListen_IT(&hi2c1);

	while (true)
	{
		float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
		last_time = osKernelGetTickCount();

		osDelayUntil(last_time + 1000); // 1 Hz
	}
}