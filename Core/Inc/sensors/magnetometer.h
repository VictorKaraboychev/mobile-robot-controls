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

struct MagnetometerData
{
	Eigen::Vector3f magnetic_orientation; // x, y, z (rad)

	bool active;
	bool data_ready;
};

extern MagnetometerData magnetometer_data;

extern Sensor magnetometer;

void StartMagTask(void *argument);

#endif /* __MAGNETOMETER_H__ */