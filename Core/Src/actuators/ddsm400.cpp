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

uint8_t LIN_PID(uint8_t id)
{
	if (id > 0x3F)
	{
		return 0x00;
	}

	uint8_t bits[6];

	for (uint8_t i = 0; i < 6; i++)
	{
		bits[i] = (id >> i) & 0x01;
	}

	uint8_t p0 = (bits[0] ^ bits[1] ^ bits[2] ^ bits[4]) & 0x01;
	uint8_t p1 = ~((bits[1] ^ bits[3] ^ bits[4] ^ bits[5]) & 0x01);

	uint8_t pid = id | (p0 << 6) | (p1 << 7);

	return pid;
}

HAL_StatusTypeDef LIN_Write(uint8_t id, uint8_t *data, uint8_t len)
{
	// Sync, ID, Reg, Data, CRC
	uint8_t bytes[11] = {0x55, id, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	for (uint8_t i = 0; i < len; i++)
	{
		bytes[i + 2] = data[i];
	}

	bytes[10] = crc8(bytes + 1, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00);
	bytes[1] = LIN_PID(bytes[1]);

	// print bytes
	for (uint8_t i = 0; i < 11; i++)
	{
		printf("0x%02X ", bytes[i]);
	}
	printf("\n");

	// Send break
	HAL_LIN_SendBreak(&huart4);

	HAL_StatusTypeDef status = UART_Write(&huart4, &uart4MutexHandle, bytes, 11);

	osDelay(10);

	return status;
}

DDSM400::DDSM400(uint8_t id)
{
	// uint8_t bytes[10] = {0xAA, 0x55, 0x53, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// uint8_t crc8_test = crc8(bytes, 9, DDSM400_CRC_POLY, DDSM400_CRC_INIT, true, true, 0x00); // 0x92
	// printf("CRC-8/MAXIM: 0x%02X\n", crc8_test);

	this->id = id;
}

DDSM400::~DDSM400()
{
}

void DDSM400::init(uint8_t id)
{
	uint8_t data[3] = {0x55, 0x53, id};

	HAL_StatusTypeDef status = LIN_Write(0xAA, data, 3);

	this->id = id;
}

void DDSM400::setMode(DDSM400_MODE mode)
{
	uint8_t data[2] = {0xA0, (uint8_t)mode};

	HAL_StatusTypeDef status = LIN_Write(id, data, 2);

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
	uint8_t data[8] = {0x64, 0, 0, 0, 0, 0, 0, 0};

	int16_t speed_int = (speed * (30 / M_PI)) * 10;					// rpm
	uint8_t acceleration_int = 1000 / (acceleration * (30 / M_PI)); // ms/rpm

	// Speed
	data[2] = speed_int & 0xFF;		   // LSB
	data[1] = (speed_int >> 8) & 0xFF; // MSB

	// Acceleration
	data[5] = acceleration_int;

	// Brake
	data[6] = brake ? 0xFF : 0x00;

	HAL_StatusTypeDef status = LIN_Write(id, data, 8);
}
