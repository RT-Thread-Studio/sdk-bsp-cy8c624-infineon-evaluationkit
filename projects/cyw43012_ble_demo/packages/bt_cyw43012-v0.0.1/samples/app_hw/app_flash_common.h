/*******************************************************************************
 * File Name: app_serial_flash.h
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library
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

#ifndef __APP_SERIAL_FLASH_H_
#define __APP_SERIAL_FLASH_H_
/*******************************************************************************
 * Header files
 ******************************************************************************/

#include "mtb_kvstore.h"
#include "cy_smif_memslot.h"

#define AUXILIARY_FLASH_BLOCK                (1)
#define AUXILIARY_FLASH_LENGTH               (16)

#ifndef USE_INTERNAL_FLASH
/* For external flash */
#define  QSPI_BUS_FREQ                       (50000000l)
#define  QSPI_GET_ERASE_SIZE                 (0)

/* SMIF configuration structure */
extern cy_stc_smif_mem_config_t*            smifMemConfigs[];

#endif
/* For internal flash */

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void app_kvstore_bd_config(mtb_kvstore_bd_t* device);
void app_kvstore_bd_init(void);
void get_kvstore_init_params(uint32_t *length, uint32_t *start_addr);

#endif //__APP_SERIAL_FLASH_H_

/* END OF FILE [] */
