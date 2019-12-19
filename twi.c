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
/**
 * @file
 * @brief File with example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * @sa twi_master_with_twis_slave_example
 */

/**
 * @defgroup twi_master_with_twis_slave_example Example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * This code presents the usage of two drivers:
 * - @ref nrf_twi_drv (in synchronous mode)
 * - @ref nrf_twis_drv (in asynchronous mode)
 *
 * On the slave, EEPROM memory is simulated.
 * The size of the simulated EEPROM is configurable in the config.h file.
 * Default memory value of the device is 320 bytes. It is simulated using internal RAM.
 * This RAM area is accessed only by simulated EEPROM so the rest of the application can access it
 * only using TWI commands via hardware configured pins.
 *
 * The selected memory chip uses a 7-bit constant address. Word to access is selected during
 * a write operation: first byte sent is used as the current address pointer.
 *
 * A maximum of an 8-byte page can be written in a single access.
 * The whole memory can be read in a single access.
 *
 * When the slave (simulated EEPROM) is initialized, it copies the given part of flash
 * (see EEPROM_SIM_FLASH_ADDRESS in config.h) into RAM (enabling the use of the slave for
 * bootloader use cases).
 *
 * Many variables like length of sequential writes/reads, TWI instance to use, endianness of
 * of slave bype addressing can be configured in config.h file
 *
 * Differences between real chip and simulated one:
 * 1. Simulated chip has practically 0 ms write time.
 *    This example does not poll the memory for readiness.
 * 2. During sequential read, when memory end is reached, there is no support for roll-over.
 *    It is recommended for master to assure that it does not try to read more than the page limits.
 *    If the master is not tracking this and trying to read after the page ends, then the slave will start to NACK
 *    for trying to over-read the memory. The master should end the transaction when slave starts NACKing, which could
 *    mean that the master has read the end of the page.
 * 3. It is possible to write a maximum of EEPROM_SIM_SEQ_WRITE_MAX_BYTES bytes in a single
 *    sequential write. However, in simulated EEPROM the whole address pointer is incremented.
 *    In a real device, writing would roll-over in memory page.
 *
 * On the master side, we communicate with that memory and allow write and read.
 * Simple commands via UART can be used to check the memory.
 *
 * Pins to short:
 * - @ref TWI_SCL_M - @ref EEPROM_SIM_SCL_S
 * - @ref TWI_SDA_M - @ref EEPROM_SIM_SDA_S
 *
 * Supported commands will be listed after Tab button press.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "config.h"
#include "eeprom_simulator.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

/**
 * @brief Write data to simulated EEPROM.
 *
 * Function uses the TWI interface to write data into EEPROM memory.
 *
 * @param     addr  Start address to write.
 * @param[in] pdata Pointer to data to send.
 * @param     size  Byte count of data to send.
 * @attention       Maximum number of bytes that may be written is @ref EEPROM_SIM_SEQ_WRITE_MAX.
 *                  In sequential write, all data must be in the same page
 *                  (higher address bits do not change).
 *
 * @return NRF_SUCCESS or reason of error.
 *
 * @attention If you wish to communicate with real EEPROM memory chip, check its readiness
 * after writing the data.
 */
static ret_code_t eeprom_write(uint16_t addr, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    /* Memory device supports only a limited number of bytes written in sequence */
    if (size > (EEPROM_SIM_SEQ_WRITE_MAX_BYTES))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    /* All written data must be in the same page */
    if ((addr / (EEPROM_SIM_SEQ_WRITE_MAX_BYTES)) != ((addr + size - 1) / (EEPROM_SIM_SEQ_WRITE_MAX_BYTES)))
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    do
    {
        uint8_t buffer[EEPROM_SIM_ADDRESS_LEN_BYTES + EEPROM_SIM_SEQ_WRITE_MAX_BYTES]; /* Addr + data */

        memcpy(buffer, &addr, EEPROM_SIM_ADDRESS_LEN_BYTES);
        memcpy(buffer + EEPROM_SIM_ADDRESS_LEN_BYTES, pdata, size);
        ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_SIM_ADDR, buffer, size + EEPROM_SIM_ADDRESS_LEN_BYTES, false);
    }while (0);
    return ret;
}


/**
 * @brief Read data from simulated EEPROM.
 *
 * Function uses the TWI interface to read data from EEPROM memory.
 *
 * @param     addr  Start address to read.
 * @param[in] pdata Pointer to the buffer to fill with data.
 * @param     size  Byte count of data to read.
 *
 * @return NRF_SUCCESS or reason of error.
 */
static ret_code_t eeprom_read(uint16_t addr, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
    if (size > (EEPROM_SIM_SIZE))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    do
    {
       uint16_t addr16 = addr;
       ret = nrf_drv_twi_tx(&m_twi_master, EEPROM_SIM_ADDR, (uint8_t *)&addr16, EEPROM_SIM_ADDRESS_LEN_BYTES, true);
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, EEPROM_SIM_ADDR, pdata, size);
    }while (0);
    return ret;
}


/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with simulated EEPROM.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }
    return ret;
}

static void twi_master_uninit(void)
{
    ret_code_t ret;
    nrf_drv_twi_disable(&m_twi_master);
    nrfx_twi_uninit(&m_twi_master.u.twi);
}

/**
 *  The beginning of the journey
 */
extern volatile bool Notify_Flag;
extern volatile uint8_t Notify_buffer[4]; 
static uint8_t buffer[4];
void twi_read(void)
{
				ret_code_t err_code;
				err_code = eeprom_simulator_init();
				APP_ERROR_CHECK(err_code);

				/* Initializing TWI master interface for EEPROM */
				err_code = twi_master_init();
				APP_ERROR_CHECK(err_code);
	
				err_code = eeprom_read(0, (uint8_t *)Notify_buffer, 4);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_RAW_INFO("EEPROM read error detected.\n");
        }
				else NRF_LOG_RAW_INFO("EEPROM read ok%2x,%2x,%2x,%2x\n",\
						Notify_buffer[0],Notify_buffer[1],Notify_buffer[2],Notify_buffer[3]);
				Notify_Flag = 1;
				twi_master_uninit();
				eeprom_simulator_uninit();
				
}

void twi_write(uint8_t * pdata)
{
				ret_code_t err_code;
				err_code = eeprom_simulator_init();
				APP_ERROR_CHECK(err_code);
				/* Initializing TWI master interface for EEPROM */
				err_code = twi_master_init();
				APP_ERROR_CHECK(err_code);
	
				err_code = eeprom_write(0, pdata, 4);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_RAW_INFO("EEPROM transmission error detected.\n");
        }
				else NRF_LOG_RAW_INFO("EEPROM transmission ok %2x,%2x,%2x,%2x\n",pdata[0],pdata[1],pdata[2],pdata[3]);
				memcpy((void*) Notify_buffer,pdata,4);
				memcpy((void*) buffer,pdata,4);
				Notify_Flag = 1;
				twi_master_uninit();
				eeprom_simulator_uninit();				
}

void twi_update(void)
{
				ret_code_t err_code;
				buffer[3]++;
				err_code = eeprom_simulator_init();
				APP_ERROR_CHECK(err_code);
				/* Initializing TWI master interface for EEPROM */
				err_code = twi_master_init();
				APP_ERROR_CHECK(err_code);
	
				err_code = eeprom_write(0, buffer, 4);
        if (NRF_SUCCESS != err_code)
        {
            NRF_LOG_INFO("EEPROM update error detected.\n");
        }
				else NRF_LOG_INFO("EEPROM update ok%2x,%2x,%2x,%2x\n",\
						buffer[0],buffer[1],buffer[2],buffer[3]);
				memcpy((void*) Notify_buffer,buffer,4);
				Notify_Flag = 1;
				twi_master_uninit();
				eeprom_simulator_uninit();				
}
/** @} */ /* End of group twi_master_with_twis_slave_example */
