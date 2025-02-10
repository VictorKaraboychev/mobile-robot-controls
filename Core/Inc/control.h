#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"

#include "utility/vector.h"
#include "utility/matrix.h"
#include "extended_kalman_filter.h"
#include "pure_pursuit.h"

#include "sensors.h"

#include <stdio.h>

#define TARGET_SPEED 0.5f // m/s
#define MAX_SPEED 1.5f	  // m/s
struct RobotState
{
	Vector position;			// x, y (m)
	Vector velocity;			// v_x, v_y (m/s)
	Vector acceleration;		// a_x, a_y (m/s^2)
	float orientation;			// θ (rad)
	float angular_velocity;		// ω (rad/s)
	float angular_acceleration; // α (rad/s^2)
};

extern RobotState robot;
extern ExtendedKalmanFilter ekf;

void UpdateIMU(const Vector &acceleration, const Vector &angular_velocity);
void UpdateMagnetometer(const Vector &orientation);
void UpdateEncoders(const float &forward_velocity, const float &angular_velocity);

void StartFusionTask(void *argument);
void StartControlTask(void *argument);

#endif /* __CONTROL_H__ */