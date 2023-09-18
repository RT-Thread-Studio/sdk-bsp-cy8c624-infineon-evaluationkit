/*******************************************************************************
 * File Name: app_internal_aux_flash.c
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

/*******************************************************************************
 * Header files
 ******************************************************************************/
#ifdef USE_INTERNAL_FLASH

#include "cyhal_flash.h"
#include "mtb_kvstore.h"
#include "cy_retarget_io.h"
#include "app_flash_common.h"

cy_rslt_t result;

static cyhal_flash_t flash_obj;
static cyhal_flash_block_info_t flash_block_info;
cyhal_flash_info_t flash_info;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
static uint32_t bd_read_size(void* context, uint32_t addr);
static uint32_t bd_program_size(void* context, uint32_t addr);
static uint32_t bd_erase_size(void* context, uint32_t addr);

static cy_rslt_t bd_read(void* context, uint32_t addr,
                         uint32_t length, uint8_t* buf);
static cy_rslt_t bd_program(void* context, uint32_t addr,
                            uint32_t length, const uint8_t* buf);
static cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length);

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/

/**
 * Function Name: bd_read_size
 *
 * Function Description:
 *   @brief Function to get the read size of the block device
 *          for a specific address.
 *
 *   @param  void* context : Context object that is passed into mtb_kvstore_init
             uint32_t addr : Address for which the read size is queried.
                           This address is passed in as start_addr + offset.
 *
 *
 *   @return uint32_t: Read size of the memory device.
 *
 */
static uint32_t bd_read_size(void* context, uint32_t addr)
{
    (void)context;
    (void)addr;
    return 1;
}

/**
 * Function Name: bd_program_size
 *
 * Function Description:
 *   @brief Function to get the program size of the block device
 *          for a specific address.
 *
 *   @param  void* context: Context object that is passed into mtb_kvstore_init
             uint32_t addr: Address for which the program size is queried.
                          This address is passed in as start_addr + offset.
 *
 *
 *   @return uint32_t     : Program size of the memory device.
 *
 */
static uint32_t bd_program_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.page_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/**
 * Function Name: bd_erase_size
 *
 * Function Description:
 *   @brief Function prototype to get the erase size of the block device
 *          for a specific address.
 *
 *   @param  void* context: Context object that is passed into mtb_kvstore_init
             uint32_t addr: Address for which the program size is queried.
                          This address is passed in as start_addr + offset.
 *
 *
 *   @return uint32_t     : Erase size of the memory device.
 *
 */
static uint32_t bd_erase_size(void* context, uint32_t addr)
{
    (void)context;
    if(flash_info.block_count != 0)
        return flash_block_info.sector_size;
    else
        return CY_RSLT_TYPE_ERROR;
}

/*******************************************************************************
 * Function Name: bd_read
 *******************************************************************************
 * Function Description:
 *   @brief Function for reading data from the block device.
 *
 *   @param  void* context : Context object that is passed into mtb_kvstore_init
             uint32_t addr: Address to read the data from the block device.
                          This address is passed in as start_addr + offset
             uint32_t length : Length of the data to be read into the buffer
             uint8_t* buf : Buffer to read the data.
 *
 *
 *   @return cy_rslt_t: Result of the read operation.
 *
 ******************************************************************************/
static cy_rslt_t bd_read(void* context, uint32_t addr,
                         uint32_t length, uint8_t* buf)
{
    (void)context;
    return cyhal_flash_read(&flash_obj, addr, buf, length);
}

/**
 * Function Name: bd_program
 *
 * Function Description:
 *   @brief Function for programming of a block device for a specific address
 *
 *   @param void* context  : Context object that is passed into mtb_kvstore_init
 *   @param uint32_t addr  : Address to program the data into the block device.
 *          This address is passed in as start_addr + offset
 *   @param int32_t length : Length of the data to be written
 *   @param uint8_t* buf   : Data that needs to be written
 *
 *   @return cy_rslt_t     : Result of the program operation.
 *
 */
static cy_rslt_t bd_program(void* context, uint32_t addr,
                            uint32_t length, const uint8_t* buf)
{
    (void)context;
    uint32_t prog_size = bd_program_size(context, addr);
    CY_ASSERT(0 == (length % prog_size));
    volatile cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length;
         loc += prog_size, buf += prog_size)
    {
        result = cyhal_flash_program(&flash_obj, loc, (const uint32_t*)buf);
    }
    return result;
}

/**
 * Function Name: bd_erase
 *
 * Function Description:
 *   @brief Function for erasing of a block device for a specific address
 *
 *   @param context        : Context object that is passed into mtb_kvstore_init
     @param uint32_t addr  : Address to erase the data from the device.
            This address is passed in as start_addr + offset
     @param uint32_t length: Length of the data that needs to be erased
 *
 *
 *   @return cy_rslt_t     : Result of the erase operation.
 *
 */
static cy_rslt_t bd_erase(void* context, uint32_t addr, uint32_t length)
{
    (void)context;
    uint32_t erase_size = bd_erase_size(context, addr);
    CY_ASSERT(0 == (length % erase_size));
    cy_rslt_t result = CY_RSLT_SUCCESS;
    for(uint32_t loc = addr; result == CY_RSLT_SUCCESS && loc < addr + length; loc += erase_size)
    {
        result = cyhal_flash_erase(&flash_obj, loc);
    }
    return result;
}
/**
 * Function Name: app_kvstore_bd_config
 *
 * Function Description:
 *   @brief  This function provides the pointer to the implemented
 *           prototype function for the block device.
 *
 *   @param mtb_kvstore_bd_t : Block device interface
 *
 *   @return None
 *
 */
void app_kvstore_bd_config(mtb_kvstore_bd_t* device)
{
    device->read         = bd_read;
    device->program      = bd_program;
    device->erase        = bd_erase;
    device->read_size    = bd_read_size;
    device->program_size = bd_program_size;
    device->erase_size   = bd_erase_size;
    device->context      = &flash_obj;
}

/**
 * Function Name: app_kvstore_bd_init
 *
 * Function Description:
 *   @brief  This function initializes the underlying block device
 *           (in this case internal flash).
 *
 *   @param void
 *   @return None
 *
 */
void app_kvstore_bd_init(void)
{
    result = cyhal_flash_init(&flash_obj);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("Internal flash initialization failed! \n");
        CY_ASSERT(0);
    }

    cyhal_flash_get_info(&flash_obj, &flash_info);
    flash_block_info = flash_info.blocks[AUXILIARY_FLASH_BLOCK];
}

void get_kvstore_init_params(uint32_t* length, uint32_t* start_addr)
{
    *length = flash_block_info.page_size * AUXILIARY_FLASH_LENGTH;

    *start_addr = flash_block_info.start_address +
                  flash_block_info.size - (*length);
}

#endif
/* END OF FILE [] */
