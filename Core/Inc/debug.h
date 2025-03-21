#ifndef __DEBUG_H__
#define __DEBUG_H__

#include "main.h"
#include "cmsis_os.h"

#include "tim.h"

#include "control.h"

#include "pwm.h"

#include "barometer.h"
#include "encoders.h"
#include "imu.h"
#include "magnetometer.h"

#include <stdio.h>

#define SLOW_BLINK blink({500, 500}, 1000)
#define FAST_BLINK blink({100, 100}, 200)

#define BLINK_1 blink({200, 1000}, 1200)
#define BLINK_2 blink({200, 150, 200, 1000}, 1550)
#define BLINK_3 blink({200, 150, 200, 150, 200, 1000}, 1900)
#define BLINK_4 blink({200, 150, 200, 150, 200, 150, 200, 1000}, 2250)
#define BLINK_5 blink({200, 150, 200, 150, 200, 150, 200, 150, 200, 1000}, 2600)

#define OFF_POWER 0.0
#define LOW_POWER 0.05
#define MEDIUM_POWER 0.2
#define HIGH_POWER 0.5
#define MAX_POWER 1.0

enum PrintLevel
{
	PL_DEBUG = 0,
	PL_INFO = 1,
	PL_WARNING = 2,
	PL_ERROR = 3
};

extern PrintLevel print_level;

bool blink(std::initializer_list<uint16_t> periods, uint32_t duration);
void setBuzzer(float power);
void StartDebugTask(void *argument);

#endif /* __CONTROL_H__ */