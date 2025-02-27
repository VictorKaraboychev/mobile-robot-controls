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

HAL_StatusTypeDef writeDDSM400(uint8_t id, uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t bytes[10] = {id, reg, 0, 0, 0, 0, 0, 0, 0, 0};

	for (uint8_t i = 0; i < len; i++)
	{
		bytes[i + 2] = data[i];
	}

	bytes[9] = crc8(bytes, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	// print bytes
	for (uint8_t i = 0; i < 10; i++)
	{
		printf("0x%02X ", bytes[i]);
	}
	printf("\n");

	HAL_LIN_SendBreak(&huart4);
	HAL_StatusTypeDef status = UART_Write(&huart4, &uart4MutexHandle, bytes, 10);

	osDelay(10);

	return status;
}

HAL_StatusTypeDef readDDSM400(uint8_t id, uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t bytes[10] = {id, reg, 0, 0, 0, 0, 0, 0, 0, 0};

	bytes[9] = crc8(bytes, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);

	HAL_StatusTypeDef status = UART_Write(&huart4, &uart4MutexHandle, bytes, 10);

	if (status != HAL_OK)
	{
		return status;
	}

	HAL_LIN_SendBreak(&huart4);
	status = UART_Read(&huart4, &uart4MutexHandle, data, len);

	return status;
}

DDSM400::DDSM400(uint8_t id)
{
	// uint8_t bytes[10] = {0xAA, 0x55, 0x53, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// uint8_t crc8_test = crc8(bytes, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);
	// printf("CRC-8/MAXIM: 0x%02X\n", crc8_test);

	this->id = id;
}

DDSM400::~DDSM400()
{
}

void DDSM400::init(uint8_t id)
{
	uint8_t data[2] = {0x53, id};

	HAL_StatusTypeDef status = writeDDSM400(0xAA, 0x55, data, 2);

	this->id = id;
}

void DDSM400::setMode(DDSM400_MODE mode)
{
	HAL_StatusTypeDef status = writeDDSM400(id, 0xA0, (uint8_t *)&mode, 1);

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
	uint8_t data[7] = {0, 0, 0, 0, 0, 0, 0};

	int16_t speed_int = (speed * (30 / M_PI)) * 10;					// rpm
	uint8_t acceleration_int = 1000 / (acceleration * (30 / M_PI)); // ms/rpm

	// Speed
	data[1] = speed_int & 0xFF;		   // LSB
	data[0] = (speed_int >> 8) & 0xFF; // MSB

	// Acceleration
	data[4] = acceleration_int;

	// Brake
	data[5] = brake ? 0xFF : 0x00;

	HAL_StatusTypeDef status = writeDDSM400(id, 0x64, data, 4);
}

