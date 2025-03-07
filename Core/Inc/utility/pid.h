#ifndef __PID_H__
#define __PID_H__

#include <cmath>

class PID
{
public:
	PID(float kp, float ki = 0, float kd = 0, float min_output = -INFINITY, float max_output = INFINITY, float min_integral = -INFINITY, float max_integral = INFINITY)
	{
		this->kp = kp;
		this->ki = ki;
		this->kd = kd;

		this->min_output = min_output;
		this->max_output = max_output;

		this->min_integral = min_integral;
		this->max_integral = max_integral;

		this->integral = 0;
		this->last_error = 0;
	}

	~PID() {}

	float update(float setpoint, float measurement, float dt)
	{
		float error = setpoint - measurement;

		integral += error * dt;
		integral = fminf(fmaxf(integral, min_integral), max_integral);

		float derivative = (error - last_error) / dt;

		float output = kp * error + ki * integral + kd * derivative;

		output = fminf(fmaxf(output, min_output), max_output);

		last_error = error;

		return output;
	}

	void reset()
	{
		integral = 0;
		last_error = 0;
	}

private:
	float kp;
	float ki;
	float kd;

	float min_output;
	float max_output;

	float min_integral;
	float max_integral;

	float integral;
	float last_error;
};

#endif // __PID_H__