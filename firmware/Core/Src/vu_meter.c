/*
 * vu_meter.c
 *
 *  Created on: Sep 18, 2023
 *      Author: laurentf
 */

#include "vu_meter.h"

int vu_init(h_vu_t * h_vu, SPI_HandleTypeDef * hspi)
{
	h_vu->led[VU_PORTA] = 0;
	h_vu->led[VU_PORTB] = 0;

	h_vu->hspi = hspi;

	// Turn off all the LEDs in GPIOA (@0x12)
	h_vu->spi_buffer[0] = VU_HEADER_WRITE;
	h_vu->spi_buffer[1] = VU_GPIOA_ADDRESS;
	h_vu->spi_buffer[2] = 0xFF;	// Everything is OFF
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, h_vu->spi_buffer, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

	// Turn off all the LEDs in GPIOB (@0x13)
	h_vu->spi_buffer[0] = VU_HEADER_WRITE;
	h_vu->spi_buffer[1] = VU_GPIOB_ADDRESS;
	h_vu->spi_buffer[2] = 0xFF;	// Everything is OFF
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, h_vu->spi_buffer, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

	// Write 0x00 in IODIRA (@0x00)
	// Controls the direction of the data I/O.
	// When a bit is set, the corresponding pin becomes an input. When a bit is clear, the corresponding pin becomes an output.
	h_vu->spi_buffer[0] = VU_HEADER_WRITE;
	h_vu->spi_buffer[1] = VU_IODIRA_ADDRESS;
	h_vu->spi_buffer[2] = 0x00;	// Everything is output
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, h_vu->spi_buffer, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

	// Write 0x00 in IODIRB (@0x01)
	// Controls the direction of the data I/O.
	// When a bit is set, the corresponding pin becomes an input. When a bit is clear, the corresponding pin becomes an output.
	h_vu->spi_buffer[0] = VU_HEADER_WRITE;
	h_vu->spi_buffer[1] = VU_IODIRB_ADDRESS;
	h_vu->spi_buffer[2] = 0x00;	// Everything is output
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, h_vu->spi_buffer, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

	return 0;
}

int vu_led(h_vu_t * h_vu, uint8_t port, uint8_t led, uint8_t state)
{
	if (state == 0)
	{
		h_vu->led[port] &= ~(1<<led);
	}
	else
	{
		h_vu->led[port] |= (1<<led);
	}

	h_vu->spi_buffer[0] = VU_HEADER_WRITE;
	h_vu->spi_buffer[1] = VU_GPIOA_ADDRESS+port;
	h_vu->spi_buffer[2] = ~(h_vu->led[port]);

	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(h_vu->hspi, h_vu->spi_buffer, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

	return 0;
}

int vu_percent(h_vu_t * h_vu, uint8_t port, uint8_t percent)
{
	uint8_t led_max = ((uint16_t)(percent) * 8) / 100;
	if (led_max >= 8) led_max = 8;

	for (int i = 0 ; i < 8 ; i++)
	{
		vu_led(h_vu, port, i, 0);
	}

	for (int i = 0 ; i < led_max ; i++)
	{
		vu_led(h_vu, port, i, 1);
	}

	return 0;
}

int vu_blink(h_vu_t * h_vu)
{
	for (int j = 0 ; j < 2 ; j++)
	{
		for (int i = 0 ; i < 8 ; i++)
		{
			vu_led(h_vu, VU_PORTA, i, 1);
			HAL_Delay(10);
		}
		for (int i = 0 ; i < 8 ; i++)
		{
			vu_led(h_vu, VU_PORTB, i, 1);
			HAL_Delay(10);
		}

		HAL_Delay(50);

		for (int i = 0 ; i < 8 ; i++)
		{
			vu_led(h_vu, VU_PORTA, i, 0);
			HAL_Delay(10);
		}
		for (int i = 0 ; i < 8 ; i++)
		{
			vu_led(h_vu, VU_PORTB, i, 0);
			HAL_Delay(10);
		}

		HAL_Delay(50);
	}

	return 0;
}

void vu_blink_red(h_vu_t * h_vu)
{
	for (int i = 0 ; i < 7 ; i++)
	{
		vu_led(h_vu, VU_PORTA, 7, 0);
		vu_led(h_vu, VU_PORTB, 7, 0);
	}

	vu_led(h_vu, VU_PORTA, 7, 1);
}
