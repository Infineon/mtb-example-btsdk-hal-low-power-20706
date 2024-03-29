/*
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*

# Low Power Sample application

## Overview
This application demonstrates the low power mode application.

This application initializes sleep mode with WICED_SLEEP_MODE_NO_TRANSPORT, and configures the WICED_P30 (SW6 BTN1) as device wake up source.

The sleep mode could be configured as WICED_SLEEP_MODE_NO_TRANSPORT or WICED_SLEEP_MODE_TRANSPORT.

Please see the following for sleep mode definition:

    WICED_SLEEP_MODE_NO_TRANSPORT - This mode is configured as the application doesn't have HCI transport.

    WICED_SLEEP_MODE_TRANSPORT    - This mode is configured as the application has HCI tranport and uses device wake line to wake up

After the device start up, the LED D9 is on and LED D10 is blinking for 10 seconds. After 10 seconds, then the device will enter into low power mode.

In low power mode, the LED 9 and LED 10 are off.

User could change the device wake up duration by modifying the APP_WAKEUP_TIME_IN_SECONDS macro (default is 10 seconds) in the low_power_20706.h header file.

User could press SW6 BTN1 to wake up the device while the device is in low power mode. After the device wakes up, then the LED D9 is on and LED D10 is blinking for 10 seconds.

After 10 seconds, the device will enter into low power mode again.

## Instructions

To demonstrate the app, follow these steps:

1)  Before programming the CYW920706WCDEVAL board, make sure the SW5 DIP switches are in the following configuration,

    |Switch  | State |
    |--------|-------|
    | 1      |  OFF  |
    | 2      |  OFF  |
    | 3      |  ON   |
    | 4      |  OFF  |
    | 5      |  OFF  |
    | 6      |  OFF  |

    This is done so that D9 and D10 can be turned off to measure the power consumption of the chip

2)  Program the board and open the PUART serial port. You should now see the debug prints.
3)  D9 is on and D10 is blinking, device is in wakeup mode.
4)  After 10 seconds, the device should enter sleep mode, both LEDs, D9 and D10, are off.
5)  Wake up the device by pressing the user button.


*/

#include "sparcommon.h"
#include "wiced_bt_ble.h"
#include "wiced_timer.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_app_common.h"
#include "wiced_transport.h"
#include "low_power_20706.h"
#include "wiced_hal_batmon.h"
#include "wiced_platform.h"
#include "wiced_power_save.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static wiced_result_t           LP_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
uint32_t LP_sleep_allow_check_callback(wiced_sleep_poll_type_t type );
wiced_sleep_config_t sleep_config;
wiced_timer_t seconds_timer;                   /* app seconds timer */
uint32_t app_wakeup_time = 0;
uint32_t app_sleep_allowed = FALSE;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static void seconds_app_timer_cb( uint32_t arg );

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START( )
{
#ifdef WICED_BT_TRACE_ENABLE

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART ); /* Route application traces to PUART */
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_P00, 0, 0); /* Select PUART pads to avoid collision with D9 */

#endif

    WICED_BT_TRACE( "Low Power Application Start\n\r" );

    /* Register call back and configuration with stack */
    wiced_bt_stack_init( LP_management_callback, NULL, NULL );
    sleep_config.sleep_mode               = WICED_SLEEP_MODE_NO_TRANSPORT;
    sleep_config.device_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_LOW;
    sleep_config.host_wake_mode           = WICED_SLEEP_WAKE_ACTIVE_LOW;
    sleep_config.sleep_permit_handler     = LP_sleep_allow_check_callback;

    app_wakeup_time = 0;
    app_sleep_allowed = FALSE;
    wiced_init_timer( &seconds_timer, &seconds_app_timer_cb, 0, WICED_SECONDS_PERIODIC_TIMER );
    wiced_start_timer( &seconds_timer, APP_TIMEOUT_IN_SECONDS );

    wiced_sleep_configure( &sleep_config );
}

/*
Management callback for Bluetooth events
*/
wiced_result_t LP_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        /* Initialize wiced app */
        wiced_bt_app_init();

        /*Register interrupt pin for waking system from power save*/
        wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, NULL, NULL );
        wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON,WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_FALLING_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE );

        /* Set the WICED_P31 to output high to turn on LED D9 */
        wiced_hal_gpio_configure_pin(WICED_P31, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH );
        /* Set the WICED_P26 to output low to turn on LED D10 */
        wiced_hal_gpio_configure_pin(WICED_P26, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
        break;

    case BTM_DISABLED_EVT:
        break;

    default:
        break;
    }

    return result;
}

/*
Callback for application to approve power save
*/
uint32_t LP_sleep_allow_check_callback(wiced_sleep_poll_type_t type )
{
    uint32_t rc = WICED_SLEEP_FOREVER;

    switch(type)
    {
        case WICED_SLEEP_POLL_SLEEP_PERMISSION:    /**< Firmware will call this call back function to polling for permission to sleep */
            if( app_sleep_allowed )
            {
                WICED_BT_TRACE( "Sleep allowed \n\r" );
                rc = WICED_SLEEP_ALLOWED;
            }
            else
            {
                rc = WICED_SLEEP_NOT_ALLOWED;
            }
            break;
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:       /**< After this call back return WICED_SLEEP_ALLOWED,
                                                        firmware will call this call back function to polling for maximum allowed sleep duration */

            /* Set the WICED_P26 to output high to turn off LED D10 to to reduce current consumption */
            wiced_hal_gpio_set_pin_output( WICED_P26, GPIO_PIN_OUTPUT_HIGH);
            /* Set the WICED_P31 to output low to turn off LED D9 to to reduce current consumption */
            wiced_hal_gpio_configure_pin(WICED_P31, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW );
            wiced_hal_gpio_set_pin_output( WICED_P31, GPIO_PIN_OUTPUT_LOW);
            WICED_BT_TRACE( "Sleep forever \n\r" );
            rc = WICED_SLEEP_FOREVER;
        default:
            break;
    }
    return rc;
}

/* The function invoked on timeout of app seconds timer. */
void seconds_app_timer_cb( uint32_t arg )
{
    app_wakeup_time++;

    WICED_BT_TRACE( "Application wakeup time: %d s\n", app_wakeup_time );

    if(app_wakeup_time & 1)
    {
        wiced_hal_gpio_set_pin_output( WICED_P26, GPIO_PIN_OUTPUT_HIGH);
    }
    else
    {
        wiced_hal_gpio_set_pin_output( WICED_P26, GPIO_PIN_OUTPUT_LOW);
    }

    if( app_wakeup_time == APP_WAKEUP_TIME_IN_SECONDS )
    {
        /* stop the led blink timer */
        wiced_stop_timer( &seconds_timer );
        app_sleep_allowed = TRUE;
    }
}
