#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "main.h"
#include "cmsis_os.h"

#include "usart.h"

#include <Eigen/Dense>
#include "constants.h"
#include "control.h"

#include <stdio.h>

#define KALMAN_ENCODERS_MEASUREMENT_SIZE 3

struct Encoder
{
	uint64_t pulses;
	uint64_t last_pulses;
};

struct EncodersData
{
	float velocity;			// v (m/s)
	float angular_velocity; // ω (rad/s)

	Encoder left;
	Encoder right;

	bool active;
	bool data_ready;
};

extern EncodersData encoders_data;

extern Sensor encoders;

void StartEncodersTask(void *argument);

#endif /* __ENCODERS_H__ */