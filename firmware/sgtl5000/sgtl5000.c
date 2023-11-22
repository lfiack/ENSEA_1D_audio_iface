/*
 * sgtl5000.c
 *
 *  Created on: Nov 22, 2023
 *      Author: laurentf
 */

#include "sgtl5000.h"

HAL_StatusTypeDef sgtl5000_i2c_read_register(h_sgtl5000_t * h_sgtl5000, sgtl5000_registers_t reg_address, uint16_t * p_data)
{
	HAL_StatusTypeDef ret;

	uint8_t buffer[2];

	ret = HAL_I2C_Mem_Read (
			h_sgtl5000->hi2c,
			h_sgtl5000->dev_address,
			reg_address,
			I2C_MEMADD_SIZE_16BIT,
			buffer,
			2,
			100
	);

	*p_data = (buffer[0] << 8) | buffer[1];

	return ret;
}

HAL_StatusTypeDef sgtl5000_i2c_write_register(h_sgtl5000_t * h_sgtl5000, sgtl5000_registers_t reg_address, uint16_t data)
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[2];

	buffer[0] = (data >> 8) & 0xFF;
	buffer[1] = data & 0xFF;

	ret = HAL_I2C_Mem_Write(
			h_sgtl5000->hi2c,
			h_sgtl5000->dev_address,
			reg_address,
			I2C_MEMADD_SIZE_16BIT,
			buffer,
			2,
			100
	);
}
