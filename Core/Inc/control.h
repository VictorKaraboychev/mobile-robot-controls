#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"

#include "utility/vector.h"
#include "utility/matrix.h"
#include "extended_kalman_filter.h"

struct State
{
	Vector position; // x, y (m)
	Vector velocity; // v_x, v_y (m/s)
	Vector acceleration; // a_x, a_y (m/s^2)
	float orientation; // θ (rad)
	float angular_velocity; // ω (rad/s)
	float angular_acceleration; // α (rad/s^2)
};

extern State state;

extern ExtendedKalmanFilter ekf;

void StartFusionTask(void *argument);

#endif /* __CONTROL_H__ */