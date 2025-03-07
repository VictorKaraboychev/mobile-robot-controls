#ifndef __SERVO_H__
#define __SERVO_H__

#include <cmath>

#include "tim.h"

#define MICROSECONDS_PER_TICK 1

class Servo
{
public:
	Servo(TIM_HandleTypeDef *htim, uint32_t channel, float max_angle, float min_angle = 0, float max_pulse_width = 2500, float min_pulse_width = 500)
	{
		this->htim = htim;
		this->channel = channel;

		this->max_angle = max_angle;
		this->min_angle = min_angle;

		this->min_pulse_width = min_pulse_width;
		this->max_pulse_width = max_pulse_width;
	}
	~Servo() {}

	void init()
	{
		// Initialize the servo timer
		HAL_TIM_PWM_Start(this->htim, this->channel);
	}

	// Set the servo angle in degrees, velocity in degrees/s^2
	void setAngle(float angle, float velocity = 0.0f)
	{
		// Constrain the angle to the valid range
		float limited_angle = fminf(fmaxf(angle, min_angle), max_angle);

		float travel_time = (limited_angle - this->angle) / velocity;

		uint32_t last_time = osKernelGetTickCount();

		while (this->angle != limited_angle)
		{
			// Calculate the delta time
			float delta_time = (osKernelGetTickCount() - last_time) / 1000.0f;
			last_time = osKernelGetTickCount();

			this->angle += limited_angle;

			// Calculate the pulse width
			uint32_t pulse_width = min_pulse_width + (max_pulse_width - min_pulse_width) * (this->angle / max_angle);

			// Set the pulse width
			uint32_t ticks = pulse_width / MICROSECONDS_PER_TICK;
			__HAL_TIM_SetCompare(this->htim, this->channel, ticks);

			osDelay(1);
		}
	}

private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	float angle;

	float min_angle;
	float max_angle;
	float min_pulse_width;
	float max_pulse_width;
};

#endif // __SERVO_H__