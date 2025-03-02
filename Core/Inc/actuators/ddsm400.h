#ifndef __DDSM400_H__
#define __DDSM400_H__

#include "main.h"

#include "usart.h"
#include "cmsis_os.h"

#include <cstdint>
#include <stdio.h>

#define DDSM400_CRC_POLY 0x31
#define DDSM400_CRC_INIT 0x00

enum DDSM400_MODE
{
	OPEN = 0x00,
	CURRENT = 0x01,
	SPEED = 0x02,
	POSITION = 0x03,
	ENABLED = 0x08,
	DISABLED = 0x09
};

enum DDSM400_FAULT
{
	NONE = 0b00000000,
	HALL_FAULT = 0b00000001,
	OVER_CURRENT = 0b00000010,
	OVER_STALL = 0b00001000,
	OVER_TEMPERATURE = 0b00010000,
	DISCONNECTION = 0b00100000,
	OVER_UNDER_VOLTAGE = 0b01000000
};

class DDSM400
{
public:
	DDSM400(uint8_t id);
	~DDSM400();

	void init(uint8_t id, bool set = false);

	// Set the motor mode
	void setMode(DDSM400_MODE mode);
	// Get the motor mode
	DDSM400_MODE getMode();

	// Set the motor speed (rads/s) -40 to 40 rads/s
	void setSpeed(float speed, float acceleration = 5, bool brake = false);
	// Get the motor speed (rads/s)
	float getSpeed() const;

	// Set the motor position (rad) 0 to 2π rad
	void setPosition(float position);
	// Get the motor position (rad)
	float getPosition() const;

	// Set the motor current (A) -4 to 4 A
	void setCurrent(float current);
	// Get the motor current (A)
	float getCurrent() const;

	// Get the encoder position
	void getEncoder(uint32_t *odometer, uint16_t *position = nullptr);

	// Enable the motor
	void enable();
	// Disable the motor
	void disable();

	// Get the motor temperature (°C)
	uint8_t getTemperature() const;

	// Get the motor status
	DDSM400_FAULT getStatus();

private:
	uint8_t id;
	DDSM400_MODE mode;
	DDSM400_FAULT status;
	float temperature;
	float current;
	float speed;
	float position;
	float acceleration;

	uint8_t buffer_tx[10];
	uint8_t buffer_rx[10];
};

#endif /* __DDSM400_H__ */