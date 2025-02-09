#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"
#include "usart.h"

#include "utility/vector.h"
#include "utility/matrix.h"

#include "lsm6dso.h"
#include "lis2mdl.h"

#define GRAVITY 9.81f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

// ---IMU---
struct IMUData
{
	Vector acceleration;	 // x, y, z (m/s^2)
	Vector angular_velocity; // p, q, r (rad/s)

	bool active;
};

extern IMUData imu_data;

void StartIMUTask(void *argument);

// ---Magnetometer---
struct MagData
{
	Vector magnetic_orientation; // x, y, z (rad)

	bool active;
};

extern MagData mag_data;

void StartMagTask(void *argument);

// ---Encoders---
struct EncodersData
{
	float velocity;			// v (m/s)
	float angular_velocity; // ω (rad/s)

	bool active;
};

extern EncodersData encoders_data;

void StartEncodersTask(void *argument);

#endif /* __SENSORS_H__ */