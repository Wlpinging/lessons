/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef FDS_MEM_H
#define FDS_MEM_H
#include "stdint.h"
/**@brief Function for change led2 blink every 1s in slow mode
 */
int fdsmem_init(void);

/**@brief Function for fds record read
 */
uint32_t record_rd(void);

/**@brief Function for fds record write
 */
void record_write( uint32_t p_data);

/**@brief Function for fds record update last value
 */
void record_update(void);

/**@brief Function for spi record read last value
 */
uint32_t spi_read(void);

/**@brief Function for spi write read last value
 */
void spi_write(uint8_t * ptr);

/**@brief Function for spi write update last value
 */
void spi_update(void);



typedef uint32_t (* Profile_Read_handler_t) (void);
typedef void (* Profile_Write_handler_t) (uint32_t p_data);
typedef void (* Profile_Update_handler_t) (void);
typedef void (* SPI_Write_handler_t) (uint8_t * p_data);
typedef struct
{
    Profile_Read_handler_t 		fds_Read_handler; /**< Read_handler to sepcified interface. */
		Profile_Write_handler_t 	fds_Write_handler;
		Profile_Update_handler_t 	fds_Update_handler;
		Profile_Read_handler_t 		spi_Read_handler; /**< Read_handler to sepcified interface. */
		SPI_Write_handler_t 			spi_Write_handler;
		Profile_Update_handler_t 	spi_Update_handler;
	
} _nus_interface;

	
//uint32_t Command[4]={ void *(0), fds_record_read,fds_record_write,fds_record_update};


#endif


