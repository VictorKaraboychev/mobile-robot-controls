#ifndef __BAROMETER_H__
#define __BAROMETER_H__

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"

#include <Eigen/Dense>
#include "constants.h"
#include "control.h"

#include "lps22hh.h"

#include <stdio.h>

#define SEA_LEVEL_PRESSURE 1013.25f // hPa

struct BarometerData
{
	float pressure;	   // hPa
	float temperature; // °C
	float altitude;	   // m

	float reference_pressure; // hPa
	float reference_altitude; // m

	bool active;
	bool data_ready;
};

extern BarometerData barometer_data;

extern Sensor barometer;

void StartBarometerTask(void *argument);

#endif /* __BAROMETER_H__ */