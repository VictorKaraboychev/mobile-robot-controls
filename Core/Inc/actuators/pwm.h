#ifndef __PWM_H__
#define __PWM_H__

#include <cmath>

#include "tim.h"

class PWM
{
public:
	PWM(TIM_HandleTypeDef *htim, uint32_t channel, float max_duty_cycle = 1.0f, float min_duty_cycle = 0.0f)
	{
		this->htim = htim;
		this->channel = channel;

		this->max_duty_cycle = max_duty_cycle;
		this->min_duty_cycle = min_duty_cycle;
	}
	~PWM() {}

	void init(float initial_duty_cycle = 0.0f)
	{
		// Initialize the PWM timer
		HAL_TIM_PWM_Start(this->htim, this->channel);

		// Set the initial duty cycle
		this->duty_cycle = initial_duty_cycle;
	}

	// Set the PWM duty cycle
	void set(float duty_cycle)
	{
		// Constrain the duty cycle to the valid range
		float limited_duty_cycle = fminf(fmaxf(duty_cycle, min_duty_cycle), max_duty_cycle);

		// Calculate the pulse width
		uint16_t pulse_width = (uint16_t)(limited_duty_cycle * (this->htim->Init.Period + 1));

		// Set the pulse width
		__HAL_TIM_SET_COMPARE(this->htim, this->channel, pulse_width);

		this->duty_cycle = limited_duty_cycle;
	}

	// Get the PWM duty cycle
	float get() const
	{
		return this->duty_cycle;
	}

	// Reset the PWM duty cycle
	void reset()
	{
		this->set(0.0f);
	}

	// Set the maximum duty cycle
	void on()
	{
		this->set(max_duty_cycle);
	}

	// Set the minimum duty cycle
	void off()
	{
		this->set(min_duty_cycle);
	}

	// Set the pulse width in microseconds
	void setMicros(uint16_t pulse_width)
	{
		// Set the pulse width
		__HAL_TIM_SET_COMPARE(this->htim, this->channel, pulse_width);
	}

private:
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	float duty_cycle;

	float max_duty_cycle;
	float min_duty_cycle;
};

#endif // __PWM_H__