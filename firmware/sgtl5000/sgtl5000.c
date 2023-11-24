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
			HAL_MAX_DELAY		// Problems if I put other than HAL_MAX_DELAY WTF
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
			HAL_MAX_DELAY		// WTF
	);

	return ret;
}

HAL_StatusTypeDef sgtl5000_i2c_set_bit(h_sgtl5000_t * h_sgtl5000, sgtl5000_registers_t reg_address, uint16_t set_mask)
{
	HAL_StatusTypeDef ret;
	uint16_t data;
	ret = sgtl5000_i2c_read_register(h_sgtl5000, reg_address, &data);
	if (ret != HAL_OK)
	{
		return ret;
	}

	data |= set_mask;

	ret = sgtl5000_i2c_write_register(h_sgtl5000, reg_address, data);
	return ret;
}

HAL_StatusTypeDef sgtl5000_i2c_clear_bit(h_sgtl5000_t * h_sgtl5000, sgtl5000_registers_t reg_address, uint16_t clear_mask)
{
	HAL_StatusTypeDef ret;
	uint16_t data;
	ret = sgtl5000_i2c_read_register(h_sgtl5000, reg_address, &data);
	if (ret != HAL_OK)
	{
		return ret;
	}

	data &= clear_mask;

	ret = sgtl5000_i2c_write_register(h_sgtl5000, reg_address, data);
	return ret;
}

HAL_StatusTypeDef sgtl5000_init(h_sgtl5000_t * h_sgtl5000)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* Chip Powerup and Supply Configurations */

	//--------------- Power Supply Configuration----------------
	// NOTE: This next 2 Write calls is needed ONLY if VDDD is
	// Configure VDDD level to 1.8V (bits 3:0)
	// Write CHIP_LINREG_CTRL 0x????
	// OK, pas touche!
	// Power up internal linear regulator (Set bit 9)
	// Write CHIP_ANA_POWER 0x7260
	// Pas touche non plus

	// NOTE: This next Write call is needed ONLY if VDDD is
	// externally driven
	// Turn off startup power supplies to save power (Clear bit 12 and 13)
	// Write CHIP_ANA_POWER 0x4260
	uint16_t clear_mask = ~((1 << 12) | (1 << 13));
	sgtl5000_i2c_clear_bit(h_sgtl5000, SGTL5000_CHIP_ANA_POWER, clear_mask);

	// NOTE: The next Write calls is needed only if both VDDA and
	// VDDIO power supplies are less than 3.1V.
	// Enable the internal oscillator for the charge pump (Set bit 11)
	// Write CHIP_CLK_TOP_CTRL 0x0800
	// Enable charge pump (Set bit 11)
	// Write CHIP_ANA_POWER 0x4A60
	// VDDA and VDDIO = 3.3V so not necessary

	// NOTE: The next modify call is only needed if both VDDA and
	// VDDIO are greater than 3.1 V
	// Configure the charge pump to use the VDDIO rail (set bit 5 and bit 6)
	// Write CHIP_LINREG_CTRL 0x006C
	// TODO VDDA and VDDIO = 3.3V so it IS necessary
	uint16_t set_mask = (1 << 5) | (1 << 6);
	sgtl5000_i2c_set_bit(h_sgtl5000, SGTL5000_CHIP_LINREG_CTRL, set_mask);

	//---- Reference Voltage and Bias Current Configuration----
	// NOTE: The value written in the next 2 Write calls is dependent
	// on the VDDA voltage value.
	// Set ground, ADC, DAC reference voltage (bits 8:4). The value should
	// be set to VDDA/2. This example assumes VDDA = 1.8 V. VDDA/2 = 0.9 V.
	// The bias current should be set to 50% of the nominal value (bits 3:1)
	// Write CHIP_REF_CTRL 0x004E
	// TODO recalculer
	// Set LINEOUT reference voltage to VDDIO/2 (1.65 V) (bits 5:0)
	// and bias current (bits 11:8) to the recommended value of 0.36 mA
	// for 10 kOhm load with 1.0 nF capacitance
	// Write CHIP_LINE_OUT_CTRL 0x0322
	// TODO recalculer

	//------------Other Analog Block Configurations--------------
	// Configure slow ramp up rate to minimize pop (bit 0)
	// Write CHIP_REF_CTRL 0x004F
	// Enable short detect mode for headphone left/right
	// and center channel and set short detect current trip level
	// to 75 mA
	// Write CHIP_SHORT_CTRL 0x1106

	// Enable Zero-cross detect if needed for HP_OUT (bit 5) and ADC (bit 1)
	// Write CHIP_ANA_CTRL 0x0133

	//------------Power up Inputs/Outputs/Digital Blocks---------
	// Power up LINEOUT, HP, ADC, DAC
	// Write CHIP_ANA_POWER 0x6AFF
	// Power up desired digital blocks
	// I2S_IN (bit 0), I2S_OUT (bit 1), DAP (bit 4), DAC (bit 5),
	// ADC (bit 6) are powered on
	// Write CHIP_DIG_POWER 0x0073

	//----------------Set LINEOUT Volume Level-------------------
	// Set the LINEOUT volume level based on voltage reference (VAG)
	// values using this formula
	// Value = (int)(40*log(VAG_VAL/LO_VAGCNTRL) + 15)
	// Assuming VAG_VAL and LO_VAGCNTRL is set to 0.9 V and
	// 1.65 V respectively, the // left LO vol (bits 12:8) and right LO
	// volume (bits 4:0) value should be set // to 5
	// Write CHIP_LINE_OUT_VOL 0x0505
	// TODO recalculer

	/* System MCLK and Sample Clock */

	// Configure SYS_FS clock to 48 kHz
	// Configure MCLK_FREQ to 256*Fs
	// Modify CHIP_CLK_CTRL->SYS_FS 0x0002 // bits 3:2
	// Modify CHIP_CLK_CTRL->MCLK_FREQ 0x0000 // bits 1:0
	// Configure the I2S clocks in master mode
	// NOTE: I2S LRCLK is same as the system sample clock
	// Modify CHIP_I2S_CTRL->MS 0x0001 // bit 7

	/* PLL Configuration */

	/* Input/Output Routing */

	return ret;
}
