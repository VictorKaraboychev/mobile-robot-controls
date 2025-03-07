#include "ddsm400.h"

// CRC-8/MAXIM
uint8_t uint8_reverse(uint8_t val)
{
	uint8_t ret = 0;
	for (size_t i = 0; i < 8; i++)
	{
		if (val & 0x80)
		{
			ret |= (1 << i);
		}
		val <<= 1;
	}
	return ret;
}

uint8_t crc8(uint8_t const *data, size_t data_size, uint8_t poly, uint8_t init, bool refin, bool refout, uint8_t xor_out)
{
	uint8_t crc = init;
	for (size_t i = 0; i < data_size; i++)
	{
		crc = crc ^ (refin ? uint8_reverse(data[i]) : data[i]);
		for (size_t j = 0; j < 8; j++)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ poly;
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return (refout ? uint8_reverse(crc) : crc) ^ xor_out;
}

HAL_StatusTypeDef DDSM400::DDSM400_Message(uint8_t *tx)
{
	HAL_StatusTypeDef status = HAL_OK;

	this->tx[0] = this->id;

	// Copy the data into the frame
	for (uint8_t i = 0; i < 8; i++)
	{
		this->tx[i + 1] = tx[i];
	}

	// Calculate the CRC
	this->tx[9] = crc8(this->tx, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	status = UART_Write(this->huart, this->muart, this->tx, 10, 5);

	status = UART_Read(this->huart, this->muart, this->rx, 10, 5);

	// Check the status
	if (status != HAL_OK)
	{
		return status;
	}

	// Confirm the CRC
	uint8_t crc = crc8(this->rx, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	if (crc != this->rx[9])
	{
		printf("CRC Error\n");
		return HAL_ERROR;
	}

	return status;
}

DDSM400::DDSM400(UART_HandleTypeDef *huart, osMutexId_t *muart)
{
	this->huart = huart;
	this->muart = muart;

	this->mode = DDSM400_MODE::DISABLED;
	this->status = DDSM400_FAULT::NONE;
}

DDSM400::~DDSM400()
{
}

void DDSM400::init(uint8_t id, bool set)
{
	if (set)
	{
		uint8_t tx[8] = {0x55, 0x53, id, 0, 0, 0, 0, 0};

		this->id = 0xAA;

		// Command the motor to set the ID 5 times
		for (uint8_t i = 0; i < 5; i++)
		{
			HAL_StatusTypeDef status = DDSM400_Message(tx);
		}
	}

	this->id = id;
}

void DDSM400::setMode(DDSM400_MODE mode)
{
	uint8_t tx[8] = {0xA0, (uint8_t)mode, 0, 0, 0, 0, 0, 0};

	HAL_StatusTypeDef status = DDSM400_Message(tx);

	this->mode = mode;
}

DDSM400_MODE DDSM400::getMode()
{
	return mode;
}

void DDSM400::enable()
{
	this->setMode(DDSM400_MODE::ENABLED);
}

void DDSM400::disable()
{
	this->setMode(DDSM400_MODE::DISABLED);
}

void DDSM400::setVelocity(float speed, float acceleration, bool brake)
{
	if (this->mode != DDSM400_MODE::SPEED)
	{
		this->setMode(DDSM400_MODE::SPEED);
	}

	uint8_t tx[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};

	if (acceleration < 0)
	{
		acceleration = this->default_acceleration;
	}

	int16_t speed_int = speed * (300 / M_PI);					  // rpm
	uint8_t acceleration_int = (100 * M_PI) / (3 * acceleration); // ms/rpm

	// Speed
	tx[2] = speed_int & 0xFF;		 // LSB
	tx[1] = (speed_int >> 8) & 0xFF; // MSB

	// Acceleration
	tx[5] = acceleration_int;

	// Brake
	tx[6] = brake ? 0xFF : 0x00;

	HAL_StatusTypeDef status = DDSM400_Message(tx);

	this->parseRX();
}

float DDSM400::getVelocity() const
{
	return this->speed;
}

void DDSM400::setPosition(float position)
{
	if (this->mode != DDSM400_MODE::POSITION)
	{
		this->setMode(DDSM400_MODE::POSITION);
	}

	uint8_t tx[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};

	uint16_t position_int = position * (INT16_MAX / M_TWOPI);

	// Position
	tx[2] = position_int & 0xFF;		// LSB
	tx[1] = (position_int >> 8) & 0xFF; // MSB

	HAL_StatusTypeDef status = DDSM400_Message(tx);

	this->parseRX();
}

float DDSM400::getPosition()
{
	uint8_t tx[8] = {0x74, 0, 0, 0, 0, 0, 0, 0};

	HAL_StatusTypeDef status = DDSM400_Message(tx);

	this->parseRX();

	return this->position;
}

void DDSM400::setCurrent(float current)
{
	if (this->mode != DDSM400_MODE::CURRENT)
	{
		this->setMode(DDSM400_MODE::CURRENT);
	}

	uint8_t tx[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};

	int16_t current_int = current * INT16_MAX;

	// Current
	tx[2] = current_int & 0xFF;		   // LSB
	tx[1] = (current_int >> 8) & 0xFF; // MSB

	HAL_StatusTypeDef status = DDSM400_Message(tx);

	this->parseRX();
}

float DDSM400::getCurrent() const
{
	return this->current;
}

void DDSM400::setDefaultAcceleration(float acceleration)
{
	this->default_acceleration = acceleration;
}

uint8_t DDSM400::getTemperature() const
{
	return this->temperature;
}

DDSM400_FAULT DDSM400::getStatus()
{
	return this->status;
}

void DDSM400::parseRX()
{
	if (this->rx[0] != this->id)
	{
		return;
	}

	switch (this->rx[1])
	{
	case 0x65:
	{
		// Speed
		int16_t speed_rx = (this->rx[1] << 8) | this->rx[2];
		this->speed = (float)(speed_rx * (M_PI / 300));

		// Acceleration
		uint8_t acceleration_int_rx = this->rx[5];
		this->acceleration = (float)((100 * M_PI) / (3 * acceleration_int_rx));

		// Current
		uint16_t current_rx = (this->rx[3] << 8) | this->rx[4];
		this->current = (float)(current_rx / INT16_MAX);

		// Temperature
		uint8_t temperature_rx = this->rx[6];
		this->temperature = (float)temperature_rx;

		// Fault
		uint8_t fault_rx = this->rx[7];
		this->status = (DDSM400_FAULT)fault_rx;
	}
	break;
	case 0x75:
	{
		// Odometer
		int32_t odometer_rx = (this->rx[1] << 24) | (this->rx[2] << 16) | (this->rx[3] << 8) | this->rx[4];

		// Position
		uint32_t position_rx = (this->rx[5] << 8) | this->rx[6];

		this->position = fmodf((float)position_rx * (M_TWOPI / INT16_MAX) + DDSM400_POSITION_OFFSET, M_TWOPI) + (float)(odometer_rx * M_TWOPI) - DDSM400_POSITION_OFFSET;

		// Fault
		uint8_t fault_rx = this->rx[7];
		this->status = (DDSM400_FAULT)fault_rx;
	}
	break;
	}
}