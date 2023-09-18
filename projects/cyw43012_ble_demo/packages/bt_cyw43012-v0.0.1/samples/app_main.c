/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */

#include "app_flash_common.h"
#include "app_bt_bonding.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_kvstore.h"
#include <FreeRTOS.h>
#include <task.h>
#include "cycfg_bt_settings.h"
#include "wiced_bt_stack.h"
#include "cybsp_bt_config.h"
#include "cybt_platform_config.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#include "app_bt_utils.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/******************************************************************************
 * Function Definitions
 ******************************************************************************/

/**
 * Function Name : main
 *
 * Function Description :
 *   @brief Entry point to the application. Set device configuration and start
 *   BT stack initialization.  The actual application initialization will happen
 *   when stack reports that BT device is ready.
 *
 *   @param: None
 *
 *   @return: None
 */
int app_main()
{
    cy_rslt_t cy_result;
    wiced_result_t wiced_result;

    /* Initialize the board support package */

    #ifdef ENABLE_BT_SPY_LOG
    {
        cybt_debug_uart_config_t config = {
            .uart_tx_pin = CYBSP_DEBUG_UART_TX,
            .uart_rx_pin = CYBSP_DEBUG_UART_RX,
            .uart_cts_pin = CYBSP_DEBUG_UART_CTS,
            .uart_rts_pin = CYBSP_DEBUG_UART_RTS,
            .baud_rate = DEBUG_UART_BAUDRATE,
            .flow_control = TRUE};
        cybt_debug_uart_init(&config, NULL);
    }
    #else
    {
        /* Initialize retarget-io to use the debug UART port */
        cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                            CYBSP_DEBUG_UART_RX,
                            CY_RETARGET_IO_BAUDRATE);
    }
    #endif //ENABLE_BT_SPY_LOG

    printf("******************** "
           "BTSTACK RT-Thread Example "
           "************************\n");

    printf("****************** "
           "Hello Sensor Application Start "
           "********************\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /*Initialize the block device used by kv-store for performing
     * read/write operations to the flash*/
    app_kvstore_bd_config(&block_device);

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(app_bt_management_callback,
                                       &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if(WICED_BT_SUCCESS == wiced_result)
    {
        printf("Bluetooth Stack Initialization Successful \n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!! \n");
        CY_ASSERT(0);
    }
}
#include <rtthread.h>
INIT_APP_EXPORT(app_main);
/* END OF FILE [] */
