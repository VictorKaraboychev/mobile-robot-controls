#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"

#include <Eigen/Dense>
#include "constants.h"
#include "extended_kalman_filter.h"
#include "pure_pursuit.h"

#include <stdio.h>

#define SERVO_1 &htim12.Instance->CCR2
#define SERVO_2 &htim12.Instance->CCR1

#define KALMAN_STATE_SIZE 15
#define KALMAN_CONTROL_SIZE 1

#define SENSOR_COUNT 3

using EKF = ExtendedKalmanFilter<float, KALMAN_STATE_SIZE, KALMAN_CONTROL_SIZE>;

struct Sensor
{
	EKF::MeasurementFunc h;
	EKF::MeasurementJacobianFunc H;
	EKF::MeasurementCovariance R;

	EKF::MeasurementFunc z;
	std::function<bool()> ready;
};

#include "barometer.h"
#include "encoders.h"
#include "imu.h"
#include "magnetometer.h"

#include "ddsm400.h"

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

#endif /* __CONTROL_H__ */