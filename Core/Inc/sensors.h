#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"
#include "usart.h"

#include <Eigen/Dense>

#include "lsm6dso.h"
#include "lis2mdl.h"

#include "control.h"

#define GRAVITY 9.81f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define PULSE_PER_REVOLUTION 4096.0f									// pulses per revolution
#define WHEEL_RADIUS 0.037f												// m
#define WHEEL_CIRCUMFERENCE (WHEEL_RADIUS * M_TWOPI)					// m
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / PULSE_PER_REVOLUTION) // m
#define WHEEL_DISTANCE 0.180f											// m

// ---IMU---
struct IMUData
{
	Eigen::Vector3f acceleration;	 // x, y, z (m/s^2)
	Eigen::Vector3f angular_velocity; // p, q, r (rad/s)

	bool active;
};

extern IMUData imu_data;

void StartIMUTask(void *argument);

// ---Magnetometer---
struct MagData
{
	Eigen::Vector3f magnetic_orientation; // x, y, z (rad)

	bool active;
};

extern MagData mag_data;

void StartMagTask(void *argument);

// ---Encoders---
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
};

extern EncodersData encoders_data;

void StartEncodersTask(void *argument);

#endif /* __SENSORS_H__ */