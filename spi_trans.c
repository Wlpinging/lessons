/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
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
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  2 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define READ_LENGTH 				4
#define WRITE_LENGTH 				4
#define UPDATE_LENGTH				8
static uint8_t   const m_tx_buf1[READ_LENGTH] ={'R','E','A','D'};/**< TX buffer. */
static uint8_t   const m_tx_buf2[WRITE_LENGTH] ={'W','R','I','T'};/**< TX buffer. */
static uint8_t   const m_tx_buf3[UPDATE_LENGTH] ={'U','P','D','A','T','E',};/**< TX buffer. */
static uint8_t       m_tx_buf[8];
static uint8_t       m_rx_buf[8];    /**< RX buffer. */
static const uint8_t m_length = 8;        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */

extern volatile bool Notify_Flag;
extern volatile uint8_t Notify_buffer[4]; 
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;    
    if ((m_tx_buf[0] == 'R')&&(m_tx_buf[1] == 'E')&&(m_tx_buf[2] == 'A')&&(m_tx_buf[3] == 'D'))
    {
				NRF_LOG_INFO("Read completed.Received: ");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf+4, 4);	
				memcpy((void*) Notify_buffer,m_rx_buf+4,4);
				Notify_Flag = 1;
    }
		else if ((m_tx_buf[0] == 'W')&&(m_tx_buf[1] == 'R')&&(m_tx_buf[2] == 'I')&&(m_tx_buf[3] == 'T'))
    {
				NRF_LOG_INFO("Write completed. %2x,%2x,%2x,%2x",m_tx_buf[4],m_tx_buf[5],m_tx_buf[6],m_tx_buf[7]);
				memcpy((void*)Notify_buffer,m_tx_buf+4,4);
				Notify_Flag = 1;
		}
		else if ((m_tx_buf[0] == 'U')&&(m_tx_buf[1] == 'P')&&(m_tx_buf[2] == 'D')&&(m_tx_buf[3] == 'A'))
    {
				NRF_LOG_INFO("Update completed. ");
				Notify_buffer[3]++;
				Notify_Flag = 1;				
		}
}

void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");
}
void spi_test(void)
{
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, 8, m_rx_buf, m_length));
}

void spi_uninit(void)
{
		nrfx_spim_uninit(&spi.u.spim);
}

void SPI_Interface_init(void)
{
		spi_init();
		spi_test();
		spi_uninit();
}
uint32_t spi_read(void)
{
				spi_init();
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;
				memcpy(m_tx_buf,m_tx_buf1,READ_LENGTH);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, READ_LENGTH, m_rx_buf, m_length));
        while (!spi_xfer_done)
        {
            __WFE();
        }
				spi_uninit();
				return  ((m_rx_buf[4]<<24)+(m_rx_buf[5]<<16)+(m_rx_buf[6]<<8)+m_rx_buf[7]);
}
				
void spi_write(uint8_t * ptr)				
{
				spi_init();
				memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;
				memcpy(m_tx_buf,m_tx_buf2,WRITE_LENGTH);
				memcpy(m_tx_buf + 4 ,ptr, WRITE_LENGTH);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, (WRITE_LENGTH+4), m_rx_buf, m_length));
        while (!spi_xfer_done)
        {
            __WFE();
        }
				spi_uninit();
				return ;
}

void spi_update(void)
{
				spi_init();
				memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;
				memcpy(m_tx_buf,m_tx_buf3,UPDATE_LENGTH);
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, UPDATE_LENGTH, m_rx_buf, m_length));
        while (!spi_xfer_done)
        {
            __WFE();
        }
				spi_uninit();				
				return ;
}
