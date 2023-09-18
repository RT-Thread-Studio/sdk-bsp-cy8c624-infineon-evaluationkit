
/*******************************************************************************
 * File Name: app_bt_gatt_handler.c
 *
 * Description: This file contains the task that handles GATT events.
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

/*******************************************************************************
 * Header Files
 ******************************************************************************/
#include "inttypes.h"
#include <FreeRTOS.h>
#include <task.h>
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "app_bt_bonding.h"
#include "app_flash_common.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/**
 * Function Name: app_bt_gatt_event_callback
 *
 * Function Description:
 *   @brief This function handles GATT events from the BT stack.
 *
 *   @param wiced_bt_gatt_evt_t event                : LE GATT event code of one
 *          byte length
 *   @param wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
 *                                                    structures
 *
 *   @return wiced_bt_gatt_status_t                  : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                            wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;

    wiced_bt_gatt_attribute_request_t *p_attr_req = &p_event_data->attribute_request;
    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = app_bt_gatt_conn_status_cb( &p_event_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = app_bt_gatt_req_cb(p_attr_req);

            break;
            /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;
            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =                                       \
            (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;


        default:
            gatt_status = WICED_BT_GATT_SUCCESS;
               break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_req_cb
 *
 * Function Description:
 *   @brief This function handles GATT server events from the BT stack.
 *
 * @param p_attr_req             : Pointer to LE GATT connection status
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_cb (wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
             /* Attribute read request */
            gatt_status =                                                      \
            app_bt_gatt_req_read_handler(p_attr_req->conn_id,
                                                p_attr_req->opcode,
                                                &p_attr_req->data.read_req,
                                                p_attr_req->len_requested);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
             /* Attribute write request */
             gatt_status =
             app_bt_gatt_req_write_handler(p_attr_req->conn_id,
                                                 p_attr_req->opcode,
                                                 &p_attr_req->data.write_req,
                                                 p_attr_req->len_requested);

             if ((GATT_REQ_WRITE == p_attr_req->opcode) &&
                 (WICED_BT_GATT_SUCCESS == gatt_status))
             {
                 wiced_bt_gatt_write_req_t *p_write_request = &p_attr_req->data.write_req;
                 wiced_bt_gatt_server_send_write_rsp(p_attr_req->conn_id,
                                                     p_attr_req->opcode,
                                                     p_write_request->handle);
             }
             break;

        case GATT_REQ_MTU:
            gatt_status =                                                      \
            wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                              p_attr_req->data.remote_mtu,
                                              CY_BT_MTU_SIZE);
             break;

        case GATT_HANDLE_VALUE_NOTIF:
                    printf("Notification send complete\n");
             break;

        case GATT_REQ_READ_BY_TYPE:
            gatt_status =                                                      \
            app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                       p_attr_req->opcode,
                                                       &p_attr_req->data.read_by_type,
                                                       p_attr_req->len_requested);
            break;

        case GATT_HANDLE_VALUE_CONF:
            {
                printf("Indication Confirmation received \n");
                hello_sensor_state.flag_indication_sent = FALSE;
            }
             break;

        default:
                printf("ERROR: Unhandled GATT Connection Request case: %d\n",
                       p_attr_req->opcode);
                break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_conn_status_cb
 *
 * Function Description:
 *   @brief This callback function handles connection status changes.
 *
 *   @param wiced_bt_gatt_connection_status_t *p_conn_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                          : See possible
 *    status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_conn_status_cb(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    if (p_conn_status->connected)
    {
        return app_bt_gatt_connection_up(p_conn_status);
    }
    else
    {
        return app_bt_gatt_connection_down(p_conn_status);
    }
}

/**
 * Function Name: app_bt_gatt_req_read_handler
 *
 * Function Description:
 *   @brief This function handles Read Requests received from the client device
 *
 *   @param conn_id              : Connection ID
 *   @param opcode               : LE GATT request type opcode
 *   @param p_read_req           : Pointer to read request containing the handle
 *          to read
 *   @param len_req              : length of data requested
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler( uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_read_t *p_read_req,
                                    uint16_t len_req)
{

    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;


    puAttribute = app_bt_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;

    printf("read_handler: conn_id:%d Handle:%x offset:%d len:%d\n ",
            conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
    /* No need for context, as buff not allocated */
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                     opcode,
                                                     to_send,
                                                     from,
                                                     NULL);
}

/**
 * Function Name: app_bt_gatt_req_write_handler
 *
 * Function Description:
 *   @brief This function handles Write Requests received from the client device
 *
 *   @param conn_id                : Connection ID
 *   @param opcode                 : LE GATT request type opcode
 *   @param p_write_req            : Pointer to LE GATT write request
 *   @param len_req                : length of data requested
 *
 *   @return wiced_bt_gatt_status_t: See possible status codes in
 *                                   wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_write_req_t *p_write_req,
                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    printf("write_handler: conn_id:%d Handle:0x%x offset:%d len:%d\n ",
           conn_id, p_write_req->handle,
           p_write_req->offset,
           p_write_req->val_len );


    /* Attempt to perform the Write Request */

    gatt_status = app_bt_set_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("WARNING: GATT set attr status 0x%x\n", gatt_status);
    }

    return (gatt_status);
}

/**
 * Function Name : app_bt_gatt_req_read_by_type_handler
 *
 * Function Description :
 *   @brief Process read-by-type request from peer device
 *
 *   @param uint16_t conn_id                       : Connection ID
 *   @param wiced_bt_gatt_opcode_t opcode          : LE GATT request type opcode
 *   @param wiced_bt_gatt_read_by_type_t p_read_req: Pointer to read request
 *          containing the handle to read
 *  @param uint16_t len_requested                  : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t                         : LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                           wiced_bt_gatt_opcode_t opcode,
                                           wiced_bt_gatt_read_by_type_t *p_read_req,
                                           uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\n",len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = app_bt_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled =
        wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                     len_requested - used_len,
                                                     &pair_len,
                                                     attr_handle,
                                                     puAttribute->cur_len,
                                                     puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("attr not found  start_handle: 0x%04x"
               "end_handle: 0x%04x  Type: 0x%04x\n", p_read_req->s_handle,
                                                       p_read_req->e_handle,
                                                       p_read_req->uuid.uu.uuid16);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)app_bt_free_buffer);
}

/**
 * Function Name: app_bt_gatt_connection_up
 *
 * Function Description:
 *   @brief This function is invoked when connection is established
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *   codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    printf("Connected to peer device: ");
    print_bd_address(p_status->bd_addr);
    printf("Connection ID '%d' \n", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr,
           sizeof(wiced_bt_device_address_t));
#ifdef PSOC6_BLE
    /* Refer to Note 2 in Document History section of Readme.md */
    if(pairing_mode == TRUE)
    {
        app_bt_add_devices_to_address_resolution_db();
        pairing_mode = FALSE;
    }
#endif
    /* Update the adv/conn state */
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name: app_bt_gatt_connection_down
 *
 * Function Description:
 *   @brief This function is invoked when connection is disconnected
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    printf("Peer device disconnected: ");
    print_bd_address(p_status->bd_addr);

    printf("conn_id:%d reason:%s\n", p_status->conn_id,
           get_bt_gatt_disconn_reason_name(p_status->reason));

    /* Resetting the device info */
    memset(hello_sensor_state.remote_addr, 0, BD_ADDR_LEN);
    hello_sensor_state.conn_id = 0;

    /* Start advertisements after disconnection */
    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                           0,
                                           NULL);
    if(result != WICED_BT_SUCCESS)
    {
        printf("Start advertisement failed: %d\n", result);
    }
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name : app_bt_find_by_handle
 *
 * Function Description:
 *   @brief Find attribute description by handle
 *
 *   @param uint16_t handle          : handle to look up
 *
 *   @return gatt_db_lookup_table_t  : pointer containing handle data
 *
 */
gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/**
 * Function Name: app_bt_set_value
 *
 * Function Description:
 *   @brief This function handles writing to the attribute handle in the GATT
 *   database using the data passed from the BT stack. The value to write is
 *   stored in a buffer whose starting address is passed as one of the
 *   function parameters
 *
 *   @param uint16_t attr_handle : GATT attribute handle
 *   @param uint8_t  p_val       : Pointer to LE GATT write request value
 *   @param uint16_t len         : length of GATT write request
 *
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                              uint8_t *p_val,
                                              uint16_t len)
{
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;
    uint8_t *p_attr   = p_val;
    cy_rslt_t rslt;
    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Detected a matching handle in external lookup table */
            isHandleInTable = WICED_TRUE;

            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;

                switch (attr_handle)
                {
               /* By writing into Characteristic Client Configuration descriptor
                *  peer can enable or disable notification or indication */
                case HDLD_HELLO_SENSOR_NOTIFY_CLIENT_CHAR_CONFIG:
                    if (len != 2)
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    app_hello_sensor_notify_client_char_config[0] = p_attr[0];
                    peer_cccd_data[bondindex] = p_attr[0] | (p_attr[1] << 8);
                    rslt = app_bt_update_cccd(peer_cccd_data[bondindex], bondindex);
                    if (CY_RSLT_SUCCESS != rslt)
                    {
                        printf("Failed to update CCCD Value in Flash! \n");
                    }
                    else{
                        printf("CCCD value updated in Flash! \n");
                    }
                    break;

                case HDLC_HELLO_SENSOR_BLINK_VALUE:
                    if (len != 1)
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    app_hello_sensor_blink[0] = p_attr[0];
                    if (app_hello_sensor_blink[0] != 0)
                    {
                        printf("hello_sensor_write_handler:num blinks: %d\n", app_hello_sensor_blink[0]);
                        /* Blink the LED only if the peer writes a single digit */
                        if(app_hello_sensor_blink[0] < 10)
                        {
                            app_bt_led_blink(app_hello_sensor_blink[0]);
                        }
                    }
                    break;

                case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
                    gatt_status = WICED_BT_GATT_SUCCESS;
                    break;
                    
                default:
                    gatt_status = WICED_BT_GATT_INVALID_HANDLE;
                    break;
                }
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }
    if (!isHandleInTable)
    {
        /* TODO: Add code to read value for handles not contained within
         * generated lookup table. This is a custom logic that depends on the
         * application, and is not used in the current application. If the value
         * for the current handle is successfully written in the below code
         * snippet, then set the result using: res = WICED_BT_GATT_SUCCESS; */
        switch(attr_handle)
        {
            default:
                /* The write operation was not performed for the
                 * indicated handle */
                printf("Write Request to Invalid Handle: 0x%x\n", attr_handle);
                gatt_status = WICED_BT_GATT_WRITE_NOT_PERMIT;
                break;
        }
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_send_message
 *
 * Function Description:
 *   @brief Check if client has registered for notification/indication and send
 *   message if appropriate
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_send_message(void)
{
    wiced_bt_gatt_status_t status;
    printf("hello_sensor_send_message: CCCD:%d\n", app_hello_sensor_notify_client_char_config[0]);

    /* If client has not registered for indication or notification, no action */
    if(0 == app_hello_sensor_notify_client_char_config[0])
    {
        return;
    }
    else if(app_hello_sensor_notify_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        status = wiced_bt_gatt_server_send_notification(hello_sensor_state.conn_id,
                                                        HDLC_HELLO_SENSOR_NOTIFY_VALUE,
                                                        app_hello_sensor_notify_len,
                                                        app_hello_sensor_notify,
                                                        NULL);
        printf("Notification Status: %d \n", status);
    }
    else if(!hello_sensor_state.flag_indication_sent)
    {
        hello_sensor_state.flag_indication_sent = TRUE;
        status = wiced_bt_gatt_server_send_indication(hello_sensor_state.conn_id,
                                                HDLC_HELLO_SENSOR_NOTIFY_VALUE,
                                                app_hello_sensor_notify_len,
                                                app_hello_sensor_notify,
                                                NULL);
        printf("Indication Status: %d \n", status);
    }
}

/**
 * Function Name: app_bt_free_buffer
 *
 * Function Description:
 *   @brief This function frees up the memory buffer
 *
 *   @param uint8_t *p_data: Pointer to the buffer to be free
 *
 *   @return None
 */
void app_bt_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/**
 * Function Name: app_bt_alloc_buffer
 *
 * Function Description:
 *   @brief This function allocates a memory buffer.
 *
 *   @param int len: Length to allocate
 *
 *   @return None
 */
void* app_bt_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/**
 * Function Name: app_bt_gatt_increment_notify_value
 *
 * Function Description:
 *   @brief Keep number of the button pushes in the last byte of the Hello
 *   message.That will guarantee that if client reads it, it will have correct
 *   data.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_gatt_increment_notify_value(void)
{
    if(0 == app_hello_sensor_notify_client_char_config[0])
    {
        return;
    }
    /* Getting the last byte */
    int last_byte = app_hello_sensor_notify_len - 1 ;
    char c = app_hello_sensor_notify[last_byte];

    c++;
    if ((c < '0') || (c > '9'))
    {
        c = '0';
    }
    app_hello_sensor_notify[last_byte] = c;
}
