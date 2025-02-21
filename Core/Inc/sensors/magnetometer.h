#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"

#include <Eigen/Dense>
#include "constants.h"
#include "control.h"

#include "lis2mdl.h"

#include <stdio.h>

#define KALMAN_MAGNETOMETER_MEASUREMENT_SIZE 1

struct MagnetometerData
{
	Eigen::Vector3f magnetic_orientation; // x, y, z (rad)

	bool active;
	bool data_ready;

	bool is_calibrating;
};

extern MagnetometerData magnetometer_data;

extern Sensor magnetometer;

void StartMagTask(void *argument);

#endif /* __MAGNETOMETER_H__ */