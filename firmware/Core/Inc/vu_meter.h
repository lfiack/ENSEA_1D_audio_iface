/*
 * vu_meter.h
 *
 *  Created on: Sep 18, 2023
 *      Author: laurentf
 */

#ifndef INC_VU_METER_H_
#define INC_VU_METER_H_

#include "main.h"

#define VU_HEADER_WRITE 0x40
#define VU_IODIRA_ADDRESS 0x00
#define VU_IODIRB_ADDRESS 0x01
#define VU_GPIOA_ADDRESS 0x12
#define VU_GPIOB_ADDRESS 0x13
#define VU_PORTA 0
#define VU_PORTB 1

typedef struct h_vu_struct
{
	uint8_t led[2];

	SPI_HandleTypeDef * hspi;

	uint8_t spi_buffer[3];
} h_vu_t;

int vu_init(h_vu_t * h_vu, SPI_HandleTypeDef * hspi);
int vu_led(h_vu_t * h_vu, uint8_t port, uint8_t led, uint8_t state);
int vu_percent(h_vu_t * h_vu, uint8_t port, uint8_t percent);
int vu_blink(h_vu_t * h_vu);
void vu_blink_red(h_vu_t * h_vu);

#endif /* INC_VU_METER_H_ */
