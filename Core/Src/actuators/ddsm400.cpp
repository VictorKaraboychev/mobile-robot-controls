#include "ddsm400.h"

extern osMutexId_t uart4MutexHandle;

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

HAL_StatusTypeDef DDSM400_Message(uint8_t id, uint8_t *tx, uint8_t *rx = nullptr)
{
	uint8_t tx_frame[10] = {id, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	// Copy the data into the frame
	for (uint8_t i = 0; i < 8; i++)
	{
		tx_frame[i + 1] = tx[i];
	}

	// Calculate the CRC
	tx_frame[9] = crc8(tx_frame, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	// print bytes
	for (uint8_t i = 0; i < 10; i++)
	{
		printf("0x%02X ", tx_frame[i]);
	}
	printf("\n");

	HAL_StatusTypeDef status = UART_Write(&huart4, &uart4MutexHandle, tx_frame, 10, 5);

	uint8_t rx_frame[10] = {0};

	status = UART_Read(&huart4, &uart4MutexHandle, rx_frame, 10, 5);

	// Confirm the CRC
	uint8_t crc = crc8(rx_frame, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	if (crc != rx_frame[9])
	{
		status = HAL_ERROR;
	}

	// print bytes
	for (uint8_t i = 0; i < 10; i++)
	{
		printf("0x%02X ", rx_frame[i]);
	}
	printf("\n");

	if (rx != nullptr)
	{
		// Copy the data into the frame
		for (uint8_t i = 0; i < 8; i++)
		{
			rx[i] = rx_frame[i + 1];
		}
	}

	osDelay(10);

	return status;
}

DDSM400::DDSM400(uint8_t id)
{
	this->id = id;
}

DDSM400::~DDSM400()
{
}

void DDSM400::init(uint8_t id, bool set)
{
	if (set)
	{
		uint8_t tx[8] = {0x55, 0x53, id, 0, 0, 0, 0, 0};

		// Command the motor to set the ID 5 times
		for (uint8_t i = 0; i < 5; i++)
		{
			HAL_StatusTypeDef status = DDSM400_Message(0xAA, tx);
		}
	}

	this->id = id;
}

void DDSM400::setMode(DDSM400_MODE mode)
{
	uint8_t tx[8] = {0xA0, (uint8_t)mode, 0, 0, 0, 0, 0, 0};

	HAL_StatusTypeDef status = DDSM400_Message(id, tx);

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

void DDSM400::setSpeed(float speed, float acceleration, bool brake)
{
	uint8_t tx[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};
	uint8_t rx[8] = {0};

	int16_t speed_int = speed * (300 / M_PI);					   // rpm
	uint8_t acceleration_int = (100 * M_PI) / (3 * acceleration); // ms/rpm

	// Speed
	tx[2] = speed_int & 0xFF;		 // LSB
	tx[1] = (speed_int >> 8) & 0xFF; // MSB

	// Acceleration
	tx[5] = acceleration_int;

	// Brake
	tx[6] = brake ? 0xFF : 0x00;

	HAL_StatusTypeDef status = DDSM400_Message(id, tx, rx);

	// Speed
	int16_t speed_int_rx = (rx[1] << 8) | rx[2];
	this->speed = (float)(speed_int_rx * (M_PI / 300));

	// Acceleration
	uint8_t acceleration_int_rx = rx[5];
	this->acceleration = (float)((1000 * M_PI) / (3 * acceleration_int_rx));

	// Current
	uint16_t current_int_rx = (rx[3] << 8) | rx[4];
	this->current = (float)(current_int_rx / INT16_MAX);

	// Temperature
	uint8_t temperature_rx = rx[6];
	this->temperature = temperature_rx;

	// Fault
	uint8_t fault_rx = rx[7];
	this->status = (DDSM400_FAULT)fault_rx;
}

float DDSM400::getSpeed() const
{
	return this->speed;
}

void DDSM400::setPosition(float position)
{
	uint8_t tx[8] = {0x62, 0, 0, 0, 0, 0, 0, 0};
	uint8_t rx[8] = {0};

	uint16_t position_int = position * (INT16_MAX / M_TWOPI);

	// Position
	tx[2] = position_int & 0xFF;		// LSB
	tx[1] = (position_int >> 8) & 0xFF; // MSB

	HAL_StatusTypeDef status = DDSM400_Message(id, tx, rx);

	// Speed
	int16_t speed_int_rx = (rx[1] << 8) | rx[2];
	this->speed = (float)(speed_int_rx * (M_PI / 300));

	// Acceleration
	uint8_t acceleration_int_rx = rx[5];
	this->acceleration = (float)((1000 * M_PI) / (3 * acceleration_int_rx));

	// Current
	uint16_t current_int_rx = (rx[3] << 8) | rx[4];
	this->current = (float)(current_int_rx / INT16_MAX);

	// Temperature
	uint8_t temperature_rx = rx[6];
	this->temperature = temperature_rx;

	// Fault
	uint8_t fault_rx = rx[7];
	this->status = (DDSM400_FAULT)fault_rx;
}

void DDSM400::setCurrent(float current)
{
	uint8_t tx[8] = {0x63, 0, 0, 0, 0, 0, 0, 0};
	uint8_t rx[8] = {0};

	int16_t current_int = current * INT16_MAX;

	// Current
	tx[2] = current_int & 0xFF;		   // LSB
	tx[1] = (current_int >> 8) & 0xFF; // MSB

	HAL_StatusTypeDef status = DDSM400_Message(id, tx, rx);

	// Speed
	int16_t speed_int_rx = (rx[1] << 8) | rx[2];
	this->speed = (float)(speed_int_rx * (M_PI / 300));

	// Acceleration
	uint8_t acceleration_int_rx = rx[5];
	this->acceleration = (float)((1000 * M_PI) / (3 * acceleration_int_rx));

	// Current
	uint16_t current_int_rx = (rx[3] << 8) | rx[4];
	this->current = (float)(current_int_rx / INT16_MAX);

	// Temperature
	uint8_t temperature_rx = rx[6];
	this->temperature = (float)temperature_rx;

	// Fault
	uint8_t fault_rx = rx[7];
	this->status = (DDSM400_FAULT)fault_rx;
}

float DDSM400::getCurrent() const
{
	return this->current;
}

void DDSM400::getEncoder(uint32_t *odometer, uint16_t *position)
{
	uint8_t tx[8] = {0x74, 0, 0, 0, 0, 0, 0, 0};
	uint8_t rx[8] = {0};

	HAL_StatusTypeDef status = DDSM400_Message(id, tx, rx);

	// Odometer
	*odometer = (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | rx[4];

	// Position
	*position = (rx[5] << 8) | rx[6];

	// Fault
	uint8_t fault_rx = rx[7];
	this->status = (DDSM400_FAULT)fault_rx;
}

uint8_t DDSM400::getTemperature() const
{
	return this->temperature;
}

DDSM400_FAULT DDSM400::getStatus()
{
	return this->status;
}