/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      gap_service.c
 *
 *  DESCRIPTION
 *      This file defines routines for using GAP service.
 *
 *  NOTE
 *      Privacy is enabled by un-commenting __GAP_PRIVACY_SUPPORT__ in
 *      user_config.h. If Privacy is supported, the application uses
 *      resolvable random address for advertising. Peripheral privacy flag
 *      and re-connection address characteristics are supported when privacy
 *      is supported.
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <gatt.h>
#include <gatt_prim.h>
#include <mem.h>
#include <string.h>
#include <buf_utils.h>
#include <bluetooth.h>

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "app_gatt.h"
#include "gap_service.h"
#include "app_gatt_db.h"
#include "nvm_access.h"

/*=============================================================================*
 *  Private Data Types
 *============================================================================*/

/* GAP service data structure */

typedef struct
{
    /* Name length in Bytes */
    uint16  length;

    /* Pointer to hold device name used by the application */
    uint8   *p_dev_name;

#ifdef __GAP_PRIVACY_SUPPORT__

    /* Peripheral privacy flag */
    bool    peripheral_privacy_flag;

    BD_ADDR_T reconnect_address;

#endif /* __GAP_PRIVACY_SUPPORT__ */

    /* NVM offset at which HID data is stored */
    uint16  nvm_offset;

} GAP_DATA_T;


/*=============================================================================*
 *  Private Data
 *============================================================================*/

/* GAP service data instance */
static GAP_DATA_T g_gap_data;

/* Default device name - Added two for storing AD Type and Null ('\0') */ 
uint8 g_device_name[DEVICE_NAME_MAX_LENGTH + 2] = {
      AD_TYPE_LOCAL_NAME_COMPLETE, 
      'C', 'S', 'R', ' ', 
      'M', 'o', 'u', 's', 'e', '\0'};

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

/* The offset of data being stored in NVM for GAP service. This offset is 
 * added to GAP service offset to NVM region (see g_gap_data.nvm_offset) 
 * to get the absolute offset at which this data is stored in NVM
 */
#define GAP_NVM_DEVICE_NAME_LENGTH_OFFSET    (0)

#define GAP_NVM_DEVICE_NAME_OFFSET           (1)

#ifdef __GAP_PRIVACY_SUPPORT__

#define GAP_NVM_DEVICE_PERIPHERAL_FLAG_OFFSET \
                (GAP_NVM_DEVICE_NAME_OFFSET + DEVICE_NAME_MAX_LENGTH)

#define GAP_NVM_DEVICE_RECONNECTION_ADDRESS_OFFSET \
    (GAP_NVM_DEVICE_PERIPHERAL_FLAG_OFFSET + 1)

/* Reconnection address can't be a resolvable random address. So make the two
 * most significant bits of reconnection address to have a resolvable random
 * address. Refer to Volume 3, Part C, Section 10.8.2.1 for more details.
 */
#define MAKE_RECONNECTION_ADDRESS_INVALID() \
    (g_gap_data.reconnect_address.nap = BD_ADDR_NAP_RANDOM_TYPE_RESOLVABLE)

/* Number of words of NVM memory used by GAP service */

/* Reconnection address takes 4 words of NVM space */
#define GAP_SERVICE_NVM_MEMORY_WORDS \
                                (GAP_NVM_DEVICE_RECONNECTION_ADDRESS_OFFSET + 4)

#else

/* If privacy is not supported, then only device name length and device name
 * are stored in NVM
 */
#define GAP_SERVICE_NVM_MEMORY_WORDS         (1 + DEVICE_NAME_MAX_LENGTH)

#endif /* __GAP_PRIVACY_SUPPORT__ */

/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void gapWriteDeviceNameToNvm(void);
static void updateDeviceName(uint16 length, uint8 *name);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      gapWriteDeviceNameToNvm
 *
 *  DESCRIPTION
 *      This function is used to write GAP Device Name Length and Device Name 
 *      to NVM 
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void gapWriteDeviceNameToNvm(void)
{

    /* Write device name length to NVM */
    Nvm_Write(&(g_gap_data.length), sizeof(g_gap_data.length),
              g_gap_data.nvm_offset + GAP_NVM_DEVICE_NAME_LENGTH_OFFSET);

    /* Write device name to NVM
     * Typecast of uint8 to uint16 or vice-versa shall not have any side 
     * affects as both types (uint8 and uint16) take one word memory on XAP
     */
    Nvm_Write((uint16*)g_gap_data.p_dev_name, g_gap_data.length,
               g_gap_data.nvm_offset + GAP_NVM_DEVICE_NAME_OFFSET);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      updateDeviceName
 *
 *  DESCRIPTION
 *      This function updates the device name and length in gap service.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void updateDeviceName(uint16 length, uint8 *name)
{
    uint8 *p_name = g_gap_data.p_dev_name;
    
    /* Update Device Name length to the maximum of DEVICE_NAME_MAX_LENGTH  */
    if(length < DEVICE_NAME_MAX_LENGTH)
        g_gap_data.length = length;
    else
        g_gap_data.length = DEVICE_NAME_MAX_LENGTH;

    MemCopy(p_name, name, g_gap_data.length);

    /* Null terminate the device name string */
    p_name[g_gap_data.length] = '\0';

    gapWriteDeviceNameToNvm();

}

/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapDataInit
 *
 *  DESCRIPTION
 *      This function is used to initialise GAP service data 
 *      structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void GapDataInit(void)
{
    /* Skip 1st byte to move over AD Type field and point to device name */
    g_gap_data.p_dev_name = (g_device_name + 1);
    g_gap_data.length = StrLen((char *)g_gap_data.p_dev_name);

#ifdef __GAP_PRIVACY_SUPPORT__

    /* If device is not bonded, by default set privacyto TRUE */
    if(!AppIsDeviceBonded())
    {
        GapSetPeripheralPrivacyFlag(TRUE);

        /* Remove the stored reconnection address. */
        GapSetReconnectionAddress(NULL);
    }

#endif /* __GAP_PRIVACY_SUPPORT__ */

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapHandleAccessRead
 *
 *  DESCRIPTION
 *      This function handles read operation on GAP service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void GapHandleAccessRead(GATT_ACCESS_IND_T *p_ind)
{
    uint16 length = 0;
    uint8  *p_value = NULL;
    sys_status rc = sys_status_success;

    switch(p_ind->handle)
    {

        case HANDLE_DEVICE_NAME:
        {
            /* Validate offset against length, it should be less than 
             * device name length
             */
            if(p_ind -> offset < g_gap_data.length)
            {
                length = g_gap_data.length - p_ind -> offset;
                p_value = (g_gap_data.p_dev_name + p_ind -> offset);
            }
            else
            {
                rc = gatt_status_invalid_offset;
            }
        }
        break;

#ifdef __GAP_PRIVACY_SUPPORT__

        case HANDLE_PERIPHERAL_PRIVACY_FLAG:
        {
            /* Peripheral privacy flag value is one byte in length. */
            length = 1;
            /* Respond with the present status of peripheral privacy flag. */
            p_value = (uint8 *)&g_gap_data.peripheral_privacy_flag;
            
        }
        break;

#endif /* __GAP_PRIVACY_SUPPORT__ */

        default:
            /* Let firmware handle the request */
            rc = gatt_status_irq_proceed;
        break;

    }

    GattAccessRsp(p_ind->cid, p_ind->handle, rc,
                  length, p_value);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapHandleAccessWrite
 *
 *  DESCRIPTION
 *      This function handles write operation on GAP service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void GapHandleAccessWrite(GATT_ACCESS_IND_T *p_ind)
{
    sys_status rc = sys_status_success;

    switch(p_ind->handle)
    {

        case HANDLE_DEVICE_NAME:
            /* Update device name */
            updateDeviceName(p_ind->size_value, p_ind->value);
        break;

#ifdef __GAP_PRIVACY_SUPPORT__

        case HANDLE_PERIPHERAL_PRIVACY_FLAG:
            GapSetPeripheralPrivacyFlag(BufReadUint8(&p_ind->value));
        break;

        case HANDLE_RECONNECTION_ADDRESS:
            /* There is no specific application error code when the remote
             * device writes to the reconnection address when peripheral
             * privacy flag is disabled. So, send a success write response
             * but don't update the reconnection address.
             */
            if(g_gap_data.peripheral_privacy_flag)
            {
                /* Store the re-connection address only if it is valid. */
                if(GapIsReconnectionAddressValid())
                {
                    GapSetReconnectionAddress(p_ind->value);
                }
            }
        break;
            
#endif /* __GAP_PRIVACY_SUPPORT__ */

        default:
        {
            /* No more IRQ characteristics */
            rc = gatt_status_write_not_permitted;
        }
        break;

    }

    GattAccessRsp(p_ind->cid, p_ind->handle, rc, 0, NULL);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapReadDataFromNVM
 *
 *  DESCRIPTION
 *      This function is used to read GAP specific data store in NVM
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void GapReadDataFromNVM(uint16 *p_offset)
{

    g_gap_data.nvm_offset = *p_offset;

    /* Read Device Length */
    Nvm_Read(&g_gap_data.length, sizeof(g_gap_data.length),
                     g_gap_data.nvm_offset + GAP_NVM_DEVICE_NAME_LENGTH_OFFSET);

    /* Typecast of uint8 to uint16 or vice-versa shall not have any side affects
     * as both types (uint8 and uint16) take one word memory on XAP.
     */
    Nvm_Read((uint16*)g_gap_data.p_dev_name, g_gap_data.length,
                            g_gap_data.nvm_offset + GAP_NVM_DEVICE_NAME_OFFSET);

    /* Add NULL character to terminate the device name string */
    g_gap_data.p_dev_name[g_gap_data.length] = '\0';

#ifdef __GAP_PRIVACY_SUPPORT__

    /* If device is not bonded, by default, set privacy to TRUE and make the
     * re-connection address invalid.
     */
    if(!AppIsDeviceBonded())
    {
        g_gap_data.peripheral_privacy_flag = TRUE;
        MAKE_RECONNECTION_ADDRESS_INVALID();
    }
    else /* Read the peripheral privacy flag status and re-connection address
          * from NVM.
          */
    {
        Nvm_Read((uint16*)&g_gap_data.peripheral_privacy_flag,
                 sizeof(g_gap_data.peripheral_privacy_flag),
                 g_gap_data.nvm_offset + GAP_NVM_DEVICE_PERIPHERAL_FLAG_OFFSET);
        Nvm_Read((uint16*)&g_gap_data.reconnect_address, sizeof(BD_ADDR_T),
        g_gap_data.nvm_offset + GAP_NVM_DEVICE_RECONNECTION_ADDRESS_OFFSET);
    }

#endif /* __GAP_PRIVACY_SUPPORT__ */

    *p_offset += GAP_SERVICE_NVM_MEMORY_WORDS;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapInitWriteDataToNVM
 *
 *  DESCRIPTION
 *      This function is used to write GAP specific data to NVM for 
 *      the first time during application initialization
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void GapInitWriteDataToNVM(uint16 *p_offset)
{

    /* The NVM offset at which GAP service data will be stored */
    g_gap_data.nvm_offset = *p_offset;

    gapWriteDeviceNameToNvm();

#ifdef __GAP_PRIVACY_SUPPORT__

    /* The application is booting up for the first time. So enable privacy and
     * update the same in NVM.
     */
    GapSetPeripheralPrivacyFlag(TRUE);

    GapSetReconnectionAddress(NULL);

#endif /* __GAP_PRIVACY_SUPPORT__ */

    *p_offset += GAP_SERVICE_NVM_MEMORY_WORDS;
}

#ifdef __GAP_PRIVACY_SUPPORT__

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapIsPeripheralPrivacyEnabled
 *
 *  DESCRIPTION
 *      This function returns whether peripheral privacy is enabled or not.
 * *
 *  RETURNS
 *       bool - PeripheralPrivacyFlag True / False 
 *
 *----------------------------------------------------------------------------*/
extern bool GapIsPeripheralPrivacyEnabled(void)
{
    return (g_gap_data.peripheral_privacy_flag);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapGetReconnectionAddress
 *
 *  DESCRIPTION
 *      This function is used to get the pointer to the reconnection address.
 * 
 *  RETURNS
 *       Pointer to reconnection address
 *
 *----------------------------------------------------------------------------*/

extern BD_ADDR_T *GapGetReconnectionAddress(void)
{
    return (&g_gap_data.reconnect_address);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapIsReconnectionAddressValid
 *
 *  DESCRIPTION
 *      This function is used to check whether a valid address is there in
 *      reconnection address.
 * 
 *  RETURNS
 *       True if the remote side had written a valid address to the
 *       reconnection address, false otherwise.
 *
 *----------------------------------------------------------------------------*/

extern bool GapIsReconnectionAddressValid(void)
{
    /* Two most significant bits of Reconnection address have to be zero. Refer
     * to Volume 3, Part C, Section 10.8.2.1 for more details.
     */
    
    return ((g_gap_data.reconnect_address.nap & BD_ADDR_NAP_RANDOM_TYPE_MASK)
             == BD_ADDR_NAP_RANDOM_TYPE_NONRESOLV);
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapSetPeripheralPrivacyFlag
 *
 *  DESCRIPTION
 *      This function is used to set Peripheral Privacy Flag and 
 *      write the same to NVM as well.
 *
 *  RETURNS
 *       Nothing
 *
 *----------------------------------------------------------------------------*/
extern void GapSetPeripheralPrivacyFlag(bool flag)
{
    g_gap_data.peripheral_privacy_flag = flag;

    /* Write peripheral privacy flag to NVM */
    Nvm_Write((uint16*)&g_gap_data.peripheral_privacy_flag,
                              sizeof(g_gap_data.peripheral_privacy_flag),
                              g_gap_data.nvm_offset + 
                              GAP_NVM_DEVICE_PERIPHERAL_FLAG_OFFSET);
    
    /* If peripheral flag is disabled, then reset the reconnection address. */
    if(!flag)
    {
        GapSetReconnectionAddress(NULL);
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapSetReconnectionAddress
 *
 *  DESCRIPTION
 *      This function is used to set Reconnection address and write the same to
 *      NVM as well.
 *
 *  RETURNS
 *       Nothing
 *
 *  NOTE
 *      NULL pointer is to be passed as the argument if reconnection address
 *      is to be reset.
 *
 *----------------------------------------------------------------------------*/

extern void GapSetReconnectionAddress(uint8 *p_val)
{
    if(p_val == NULL)
    {
        MAKE_RECONNECTION_ADDRESS_INVALID();
    }

    else
    {
        MemCopy(&g_gap_data.reconnect_address, (BD_ADDR_T*)p_val,
                                                        sizeof(BD_ADDR_T));        
    }
    
    /* Write re-connection address to NVM */
    Nvm_Write((uint16*)&g_gap_data.reconnect_address,
                          sizeof(BD_ADDR_T),
                          g_gap_data.nvm_offset + 
                          GAP_NVM_DEVICE_RECONNECTION_ADDRESS_OFFSET);

    /* The updated reconnection address needs to be in whitelist as the remote
     * host may connect using this address during re-connection.
     */
    AppUpdateWhiteList();
}

#endif /* __GAP_PRIVACY_SUPPORT__ */

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapCheckHandleRange
 *
 *  DESCRIPTION
 *      This function is used to check if the handle belongs to the GAP 
 *      service
 *
 *  RETURNS
 *      Boolean - Indicating whether handle falls in range or not.
 *
 *----------------------------------------------------------------------------*/

extern bool GapCheckHandleRange(uint16 handle)
{
    return ((handle >= HANDLE_GAP_SERVICE) &&
            (handle <= HANDLE_GAP_SERVICE_END))
            ? TRUE : FALSE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GapGetNameAndLength
 *
 *  DESCRIPTION
 *      This function is used to get the reference to the 'g_device_name' array,
 *      which contains AD Type and Device name. This function also returns the
 *      AD Type and Device name length.
 *
 *  RETURNS
 *      Pointer to device name array.
 *
 *----------------------------------------------------------------------------*/

extern uint8 *GapGetNameAndLength(uint16 *p_name_length)
{
    *p_name_length = StrLen((char *)g_device_name);
    return (g_device_name);
}

#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteGapServiceDataInNVM
 *
 *  DESCRIPTION
 *      This function writes the GAP service data in NVM
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/

extern void WriteGapServiceDataInNVM(void)
{
    /* Gap Service has only device name to write into NVM */
    gapWriteDeviceNameToNvm();

#ifdef __GAP_PRIVACY_SUPPORT__

    /* Write peripheral privacy flag to NVM */
    Nvm_Write((uint16*)&g_gap_data.peripheral_privacy_flag,
                       sizeof(g_gap_data.peripheral_privacy_flag),
                       g_gap_data.nvm_offset + 
                              GAP_NVM_DEVICE_PERIPHERAL_FLAG_OFFSET);

    /* Write re-connection address to NVM */
    Nvm_Write((uint16*)&g_gap_data.reconnect_address,
                       sizeof(BD_ADDR_T),
                       g_gap_data.nvm_offset + 
                          GAP_NVM_DEVICE_RECONNECTION_ADDRESS_OFFSET);


#endif /* __GAP_PRIVACY_SUPPORT__ */

}
#endif /* NVM_TYPE_FLASH */

