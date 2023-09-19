/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description: This file is the public interface of app_bt_event_handler.c
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 ******************************************************************************/

#ifndef SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_


/*******************************************************************************
 * Structures
 *******************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;  // remote peer device address
    uint32_t  timer_count_s;                // count of sec since the start of seconds timer
    uint32_t  timer_count_ms;               // count of ms since the start of ms timer
    uint16_t  conn_id;                      // connection ID of the peer
    uint16_t  peer_mtu;                     // peer MTU
    uint8_t   flag_indication_sent;         // to store the state of indication confirmation
    uint8_t   num_to_send;                  // number of messages to send. Incremented on each button interrupt   
} hello_sensor_state_t;

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
/* Holds the global state of the hello sensor application */
extern hello_sensor_state_t hello_sensor_state;

extern uint8_t bondindex;
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

wiced_result_t
app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);

void app_bt_application_init(void);

void app_bt_adv_stop_handler(void);

#endif /* SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_ */
