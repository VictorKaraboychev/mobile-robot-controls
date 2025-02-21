#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"

#include <Eigen/Dense>
#include "constants.h"
#include "control.h"

#include "lsm6dso.h"

#include <stdio.h>

#define KALMAN_ACCELEROMETER_MEASUREMENT_SIZE 5
#define KALMAN_GYROSCOPE_MEASUREMENT_SIZE 3

struct AccelerometerData
{
	Eigen::Vector3f acceleration; // x, y, z (m/s^2)

	bool active;
	bool data_ready;
};

struct GyroscopeData
{
	Eigen::Vector3f angular_velocity; // p, q, r (rad/s)

	bool active;
	bool data_ready;
};

extern AccelerometerData accelerometer_data;
extern GyroscopeData gyroscope_data;

extern Sensor accelerometer;
extern Sensor gyroscope;

void StartIMUTask(void *argument);

#endif /* __IMU_H__ */