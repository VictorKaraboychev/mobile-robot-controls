#include "debug.h"

PrintLevel print_level = PL_INFO;

bool blink(std::initializer_list<uint16_t> periods, uint32_t duration)
{
	uint32_t current_time = HAL_GetTick() % duration;

	bool state = true;

	for (auto period : periods)
	{
		if (current_time < period) // If the current time is within the period
		{
			break;
		}

		current_time -= period; // Subtract the period length
		state = !state;			// Toggle the state
	}

	return state;
}

float buzzer_power = OFF_POWER;

void setBuzzer(float power)
{
	buzzer_power = power;
}

PWM green_led1(&htim4, TIM_CHANNEL_3);
PWM green_led2(&htim4, TIM_CHANNEL_2);
PWM green_led3(&htim4, TIM_CHANNEL_1);

PWM red_led1(&htim8, TIM_CHANNEL_2);
PWM red_led2(&htim8, TIM_CHANNEL_1);
PWM red_led3(&htim4, TIM_CHANNEL_4);

PWM buzzer(&htim15, TIM_CHANNEL_1);

void StartDebugTask(void *argument)
{
	// GREEN LEDs
	green_led1.init();
	green_led2.init();
	green_led3.init();

	// RED LEDs
	red_led1.init();
	red_led2.init();
	red_led3.init();

	// BUZZER
	buzzer.init();

	bool all_sensors_active = false;

	uint32_t last_time = osKernelGetTickCount();

	// Startup loop
	while (osKernelGetTickCount() < 1000)
	{
		buzzer.set(MEDIUM_POWER * blink({500, 200, 100, 100, 100}, 1000));

		osDelay(10); // 100 Hz
	}

	buzzer.set(OFF_POWER);

	osDelay(1000);

	// Main loop
	while (true)
	{
		float elapsed_time = (osKernelGetTickCount() - last_time) / 1000.0f;

		// Print every 0.5s
		if (elapsed_time >= 0.5f && print_level <= PL_DEBUG)
		{
			// printf("[DEBUG] IMU: %s, Magnetometer: %s, Encoders: %s\n",
			// 	   imu_data.active ? "  ACTIVE" : "INACTIVE",
			// 	   magnetometer_data.active ? " ACTIVE" : "INACTIVE",
			// 	   encoders_data.active ? " ACTIVE" : "INACTIVE");

			// Print robot state
			// printf("[DEBUG] s: %.2f %.2f %.2f, v: %.2f %.2f %.2f, a: %.2f %.2f %.2f, w: %.2f %.2f %.2f, w_v: %.2f %.2f %.2f\n",
			// 	   robot.position[0], robot.position[1], robot.position[2],
			// 	   robot.velocity[0], robot.velocity[1], robot.velocity[2],
			// 	   robot.acceleration[0], robot.acceleration[1], robot.acceleration[2],
			// 	   robot.orientation[0] * RAD_TO_DEG, robot.orientation[1] * RAD_TO_DEG, robot.orientation[2] * RAD_TO_DEG,
			// 	   robot.angular_velocity[0] * RAD_TO_DEG, robot.angular_velocity[1] * RAD_TO_DEG, robot.angular_velocity[2] * RAD_TO_DEG);

			printf("[DEBUG]: s: (%.2f %.2f %.2f) v: (%.2f %.2f %.2f) t: (%.2f %.2f %.2f) \n",
				   robot.position[0],
				   robot.position[1],
				   robot.position[2],
				   robot.velocity[0],
				   robot.velocity[1],
				   robot.velocity[2],
				   robot.orientation[0] * RAD_TO_DEG,
				   robot.orientation[1] * RAD_TO_DEG,
				   robot.orientation[2] * RAD_TO_DEG);
			last_time = osKernelGetTickCount();
		}

		// Reset the LEDs and buzzer
		green_led1.set(OFF_POWER);
		green_led2.set(OFF_POWER);
		green_led3.set(OFF_POWER);

		red_led1.set(OFF_POWER);
		red_led2.set(OFF_POWER);
		red_led3.set(OFF_POWER);

		buzzer.set(OFF_POWER);

		if (!accelerometer_data.active) // If the IMU is not active
		{
			red_led1.set(MEDIUM_POWER * BLINK_1);
		}
		else if (!magnetometer_data.active) // If the magnetometer is not active
		{
			red_led1.set(MEDIUM_POWER * BLINK_2);
		}
		else if (!barometer_data.active) // If the barometer is not active
		{
			red_led1.set(MEDIUM_POWER * BLINK_3);
		}
		else if (!encoders_data.active) // If the encoders are not active
		{
			red_led1.set(MEDIUM_POWER * BLINK_4);
		}
		else // If all sensors are active
		{
			green_led1.set(MEDIUM_POWER * SLOW_BLINK);
		}

		// If the magnetometer is calibrating beep the buzzer
		if (accelerometer_data.is_calibrating || magnetometer_data.is_calibrating)
		{
			buzzer.set(LOW_POWER * FAST_BLINK);
			green_led2.set(MEDIUM_POWER * FAST_BLINK);
		}

		if (buzzer_power > OFF_POWER)
		{
			buzzer.set(buzzer_power);
		}

		osDelay(10); // 100 Hz
	}
}