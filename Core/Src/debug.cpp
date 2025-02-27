#include "debug.h"

PrintLevel print_level = PL_DEBUG;

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

void setPWM(volatile uint32_t *handle, float brightness, bool pattern = true, uint16_t resolution = 1000)
{
	*handle = (uint32_t)(pattern * brightness * resolution);
}

void StartDebugTask(void *argument)
{
	// GREEN LEDs
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	// RED LEDs
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	// BUZZER
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

	bool all_sensors_active = false;

	uint32_t last_time = HAL_GetTick();

	while (true)
	{
		float elapsed_time = (HAL_GetTick() - last_time) / 1000.0f;

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

			last_time = HAL_GetTick();
		}

		// Reset the LEDs and buzzer
		setPWM(GREEN_LED1, OFF_POWER);
		setPWM(GREEN_LED2, OFF_POWER);
		setPWM(GREEN_LED3, OFF_POWER);

		setPWM(RED_LED1, OFF_POWER);
		setPWM(RED_LED2, OFF_POWER);
		setPWM(RED_LED3, OFF_POWER);

		setPWM(BUZZER, OFF_POWER);

		// Check if all sensors are active
		all_sensors_active = accelerometer_data.active && magnetometer_data.active && barometer_data.active && encoders_data.active;

		if (all_sensors_active)
		{
			setPWM(GREEN_LED1, MEDIUM_POWER, SLOW_BLINK);
		}
		else
		{
			if (!accelerometer_data.active) // If the IMU is not active
			{
				setPWM(RED_LED1, MEDIUM_POWER, BLINK_1);
			}
			else if (!magnetometer_data.active) // If the magnetometer is not active
			{
				setPWM(RED_LED1, MEDIUM_POWER, BLINK_2);
			}
			else if (!barometer_data.active) // If the barometer is not active
			{
				setPWM(RED_LED1, MEDIUM_POWER, BLINK_3);
			}
			else if (!encoders_data.active) // If the encoders are not active
			{
				setPWM(RED_LED1, MEDIUM_POWER, BLINK_4);
			}
		}

		// If the magnetometer is calibrating beep the buzzer
		if (magnetometer_data.is_calibrating)
		{
			// setPWM(BUZZER, LOW_POWER, FAST_BLINK);
			setPWM(GREEN_LED2, MEDIUM_POWER, FAST_BLINK);
		}

		osDelay(10);
	}
}