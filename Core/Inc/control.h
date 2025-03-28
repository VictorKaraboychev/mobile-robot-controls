#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"

#include <Eigen/Dense>
#include "constants.h"
#include "extended_kalman_filter.h"
#include "pure_pursuit.h"
#include "utility.h"

#include <stdio.h>

#define KALMAN_STATE_SIZE 15
#define KALMAN_CONTROL_SIZE 1

#define SENSOR_COUNT 2

#define SERVO_STORAGE_ANGLE 40.0f
#define SERVO_DEFAULT_ANGLE 60.0f
#define SERVO_PICKUP_ANGLE 230.0f
#define SERVO_DROPOFF_ANGLE 200.0f

enum RobotMode
{
	ROBOT_MODE_DISABLED = 0,
	ROBOT_MODE_ENABLED = 1,
	ROBOT_MODE_ENABLING_TRANSITION = 2,
	ROBOT_MODE_DISABLING_TRANSITION = 3,
	ROBOT_MODE_PICKUP_TRANSITION = 4,
	ROBOT_MODE_DROPOFF_TRANSITION = 5,
};

enum RobotFunction
{
	ROBOT_FUNCTION_NONE = 0,
	ROBOT_FUNCTION_ENABLE = 1,
	ROBOT_FUNCTION_DISABLE = 2,
	ROBOT_FUNCTION_PICKUP = 3,
	ROBOT_FUNCTION_DROPOFF = 4,
	ROBOT_FUNCTION_LOAD = 5
};

using EKF = ExtendedKalmanFilter<float, KALMAN_STATE_SIZE, KALMAN_CONTROL_SIZE>;

struct Sensor
{
	EKF::MeasurementFunc h;
	EKF::MeasurementJacobianFunc H;
	EKF::MeasurementCovariance R;

	EKF::MeasurementFunc z;
	std::function<bool()> ready;
	std::function<void()> consume;
};

#include "barometer.h"
#include "encoders.h"
#include "imu.h"
#include "magnetometer.h"

#include "ddsm400.h"
#include "servo.h"
#include "pid.h"
#include "pwm.h"
#include "debug.h"

struct RobotState
{
	Eigen::Vector3f position;		  // x, y, z (m)
	Eigen::Vector3f velocity;		  // v_x, v_y, v_z (m/s)
	Eigen::Vector3f acceleration;	  // a_x, a_y, a_z (m/s^2)
	Eigen::Vector3f orientation;	  // (θ, φ, ψ) (rad)
	Eigen::Vector3f angular_velocity; // (ω_x, ω_y, ω_z) (rad/s)
};

extern RobotState robot;
extern EKF ekf;

void StartFusionTask(void *argument);
void StartControlTask(void *argument);
void StartCommTask(void *argument);

void StartLeftMotorTask(void *argument);
void StartRightMotorTask(void *argument);

#endif /* __CONTROL_H__ */