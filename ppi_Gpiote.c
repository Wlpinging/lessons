
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_drv_ppi.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
//#define PPIGPIO    1



#ifdef PPIGPIO

#else
	#define LESSON3_LED2_MEAS_INTERVAL1      APP_TIMER_TICKS(500)       /**LED2 level measurement interval (ticks). */
	#define LESSON3_LED2_MEAS_INTERVAL2      APP_TIMER_TICKS(1000)       /**LED2 level measurement interval (ticks). */
	APP_TIMER_DEF(Lesson3LedTimer);
#endif

#ifdef BSP_LED_1
    #define GPIO_OUTPUT_PIN_NUMBER BSP_LED_1  /**< Pin number for output. */
#endif
#ifndef GPIO_OUTPUT_PIN_NUMBER
    #error "Please indicate output pin"
#endif

#ifdef PPIGPIO 
static nrf_drv_timer_t timer2 = NRF_DRV_TIMER_INSTANCE(1);

void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context){}

static void led_blinking_setup()
{
    uint32_t compare_evt_addr;
    uint32_t gpiote_task_addr;
    nrf_ppi_channel_t ppi_channel;
    ret_code_t err_code;
    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    err_code = nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &config);
    APP_ERROR_CHECK(err_code);


    nrf_drv_timer_extended_compare(&timer2, (nrf_timer_cc_channel_t)0, 500 * 1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err_code);

    compare_evt_addr = nrf_drv_timer_event_address_get(&timer2, NRF_TIMER_EVENT_COMPARE0);
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel, compare_evt_addr, gpiote_task_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
}

/**
 * @brief Function for application main entry.
 */
void ppi_Setting(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&timer2, &timer_cfg, timer_dummy_handler);
    APP_ERROR_CHECK(err_code);
#ifdef NRF51
    //Workaround for PAN-73.
    *(uint32_t *)0x40008C0C = 1;
#endif

    // Setup PPI channel with event from TIMER compare and task GPIOTE pin toggle.
    led_blinking_setup();

    // Enable timer
    nrf_drv_timer_enable(&timer2);

}
#else
void Lesson3LedTimer_handler(void * p_context)
{
		nrf_gpio_pin_toggle(GPIO_OUTPUT_PIN_NUMBER);
}
void ppi_Setting(void)
{
    nrf_gpio_cfg_output(GPIO_OUTPUT_PIN_NUMBER);
		nrf_gpio_pin_write(GPIO_OUTPUT_PIN_NUMBER,0);
		app_timer_create(&Lesson3LedTimer, APP_TIMER_MODE_REPEATED, Lesson3LedTimer_handler);
		app_timer_start(Lesson3LedTimer, LESSON3_LED2_MEAS_INTERVAL1, NULL);
}

#endif

void Timer_To1Second(void)
{
		#ifdef PPIGPIO
		{
			nrf_drv_timer_disable(&timer2);
			nrf_drv_timer_extended_compare(&timer2, (nrf_timer_cc_channel_t)0, 1000 * 1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
			nrf_drv_timer_enable(&timer2);
		}
		#else
		app_timer_stop(Lesson3LedTimer);
		app_timer_create(&Lesson3LedTimer, APP_TIMER_MODE_REPEATED, Lesson3LedTimer_handler);
		app_timer_start(Lesson3LedTimer, LESSON3_LED2_MEAS_INTERVAL2, NULL);	
		#endif
}

void Timer_Disable(void)
{
		#ifdef PPIGPIO
		nrf_drv_timer_disable(&timer2);
		#else
		app_timer_stop(Lesson3LedTimer);
		#endif
}

void Timer_TohalfSecond(void)
{
		#ifdef PPIGPIO
		nrf_drv_timer_extended_compare(&timer2, (nrf_timer_cc_channel_t)0, 500 * 1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
		nrf_drv_timer_enable(&timer2);
		#else
		app_timer_stop(Lesson3LedTimer);
		app_timer_create(&Lesson3LedTimer, APP_TIMER_MODE_REPEATED, Lesson3LedTimer_handler);
		app_timer_start(Lesson3LedTimer, LESSON3_LED2_MEAS_INTERVAL1, NULL);	
		#endif
}

