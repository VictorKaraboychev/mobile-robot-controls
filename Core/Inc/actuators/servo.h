#ifndef __SERVO_H__
#define __SERVO_H__

#include <cmath>

#include "tim.h"

#define MICROSECONDS_PER_TICK 1

class Servo
{
public:
	Servo(volatile uint32_t *handle, float max_angle, float min_angle = 0, float max_pulse_width = 2500, float min_pulse_width = 500)
	{
		this->handle = handle;

		this->max_angle = max_angle;
		this->min_angle = min_angle;

		this->min_pulse_width = min_pulse_width;
		this->max_pulse_width = max_pulse_width;
	}

	~Servo() {}

	void setAngle(float angle)
	{
		// Constrain the angle to the valid range
		float limited_angle = fminf(fmaxf(angle, min_angle), max_angle);

		// Calculate the pulse width
		uint32_t pulse_width = min_pulse_width + (max_pulse_width - min_pulse_width) * (limited_angle / max_angle);

		// Set the pulse width
		*handle = pulse_width / MICROSECONDS_PER_TICK; // 250Hz
	}

private:
	volatile uint32_t *handle;
	float min_angle;
	float max_angle;
	float min_pulse_width;
	float max_pulse_width;
};

#endif // __SERVO_H__