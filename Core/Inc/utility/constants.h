#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#define GRAVITY 9.81f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define PULSE_PER_REVOLUTION 4096.0f									// pulses per revolution
#define WHEEL_RADIUS 0.037f												// m
#define WHEEL_CIRCUMFERENCE (WHEEL_RADIUS * M_TWOPI)					// m
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / PULSE_PER_REVOLUTION) // m
#define WHEEL_DISTANCE 0.180f											// m

#define TARGET_SPEED 0.5f // m/s
#define MAX_SPEED 1.5f	  // m/s

#endif /* __CONSTANTS_H__ */
