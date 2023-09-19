/*******************************************************************************
 * File Name:   app_bt_bonding.h
 *
 * Description: This is the header file for the bonding related defines for
 *              Hello Server Example for ModusToolbox.
 *
 * Related Document: See README.md
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
#ifndef __APP_BT_BONDING_H_
#define __APP_BT_BONDING_H_

/*******************************************************************************
*        Header Files
*******************************************************************************/
#include "wiced_bt_stack.h"
#include "wiced_bt_ble.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "cycfg_bt_settings.h"
#include "mtb_kvstore.h"

/*******************************************************************************
*        Macro Definitions
*******************************************************************************/
/* Max number of bonded devices */
#define  BOND_INDEX_MAX                      (4)
/* LE Key Size */
#define  KEY_SIZE_MAX                        (0x10)

/*******************************************************************************
*        Structures and Enumerations
*******************************************************************************/
/* enum for bond_info_t structure */
enum bond_info_e
{
    NUM_BONDED ,
    NEXT_FREE_INDEX
};

/* Structure to store info that goes into serial flash - it holds the number of bonded devices, remote keys and local keys */
typedef struct
{
    uint8_t slot_data[2];   /* Refer  bond_info_e */
    wiced_bt_device_link_keys_t link_keys[BOND_INDEX_MAX];
    wiced_bt_ble_privacy_mode_t privacy_mode[BOND_INDEX_MAX];
}bond_info_t;

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/

/* Kvstore block device */
extern  mtb_kvstore_bd_t  block_device;

/* Object for kvstore library */
extern mtb_kvstore_t  kvstore_obj;

/* Variable to store pairing key information */
extern bond_info_t   bond_info;

/* Variable to store CCCD values of peer device */
extern uint16_t   peer_cccd_data[BOND_INDEX_MAX];

/* Variable to store identity keys of our device */
extern wiced_bt_local_identity_keys_t identity_keys;

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
void app_kv_store_init(void);
cy_rslt_t app_bt_restore_bond_data(void);
cy_rslt_t app_bt_update_bond_data(void);
cy_rslt_t app_bt_delete_bond_info(void);
wiced_result_t app_bt_delete_device_info(uint8_t index);
cy_rslt_t app_bt_update_slot_data(void);
cy_rslt_t app_bt_save_device_link_keys(wiced_bt_device_link_keys_t *link_key);
cy_rslt_t app_bt_save_local_identity_key(wiced_bt_local_identity_keys_t id_key);
cy_rslt_t app_bt_read_local_identity_keys(void);
cy_rslt_t app_bt_update_cccd(uint16_t cccd, uint8_t index);
cy_rslt_t app_bt_restore_cccd(void);
uint8_t app_bt_find_device_in_flash(uint8_t *bd_addr);
void app_bt_add_devices_to_address_resolution_db(void);
void print_bond_data(void);


#endif // __APP_BT_BONDING_H_

/* [] END OF FILE */
