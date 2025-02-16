#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"

#include <Eigen/Dense>
#include "extended_kalman_filter.h"
// #include "pure_pursuit.h"

#include "sensors.h"

#include <stdio.h>

#define TARGET_SPEED 0.5f // m/s
#define MAX_SPEED 1.5f	  // m/s

#define KALMAN_STATE_SIZE 15
#define KALMAN_CONTROL_SIZE 1

#define KALMAN_ACCELEROMETER_MEASUREMENT_SIZE 5
#define KALMAN_GYROSCOPE_MEASUREMENT_SIZE 3
#define KALMAN_MAGNETOMETER_MEASUREMENT_SIZE 1
#define KALMAN_ENCODERS_MEASUREMENT_SIZE 3
struct RobotState
{
	Eigen::Vector3f position;		  // x, y, z (m)
	Eigen::Vector3f velocity;		  // v_x, v_y, v_z (m/s)
	Eigen::Vector3f acceleration;	  // a_x, a_y, a_z (m/s^2)
	Eigen::Vector3f orientation;	  // (θ, φ, ψ) (rad)
	Eigen::Vector3f angular_velocity; // (ω_x, ω_y, ω_z) (rad/s)
};

using EKF = ExtendedKalmanFilter<float, KALMAN_STATE_SIZE, KALMAN_CONTROL_SIZE>;

extern RobotState robot;
extern EKF ekf;

void UpdateAccelerometer(const Eigen::Vector3f &acceleration);
void UpdateGyroscope(const Eigen::Vector3f &angular_velocity);
void UpdateMagnetometer(const Eigen::Vector3f &orientation);
void UpdateEncoders(const float &forward_velocity, const float &angular_velocity);

void StartFusionTask(void *argument);
void StartControlTask(void *argument);

#endif /* __CONTROL_H__ */