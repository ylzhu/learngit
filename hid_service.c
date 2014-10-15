/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      hid_service.c
 *
 *  DESCRIPTION
 *      This file defines routines for using HID service.
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <gatt.h>
#include <gatt_prim.h>
#include <mem.h>
#include <buf_utils.h>
#include <pio.h>

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "app_gatt.h"
#include "hid_service.h"
#include "nvm_access.h"
#include "app_gatt_db.h"
#include "sensor.h"
#include "user_config.h"

/*=============================================================================*
 *  Private Data Types
 *============================================================================*/

typedef struct
{
    /* INPUT report (BOOT or Interrupt) notification handle as configured 
     * by peer host device.
     */
    uint16                  input_report_handle;

    /* Input Report Client Configuration */
    gatt_client_config      input_client_config;

    /* Input Boot Report Client Configuration */
    gatt_client_config      input_boot_client_config;

    /* HID protocol mode is hid_report_mode mode by default */
    hid_protocol_mode       report_mode;

    /* Set to TRUE if the HID device is suspended. By default set to FALSE (ie., 
     * Not Suspended)
     */
    bool                    suspended;

    /* This flag is set if the notifications for Boot or Report mode input 
     * reports are enabled and the corresponding input reporting mode is set
     */
    bool                    notify_enable;

    /* NVM offset at which HID data is stored */
    uint16                  nvm_offset;

} HID_DATA_T;

/*=============================================================================*
 *  Private Data
 *============================================================================*/

static HID_DATA_T hid_data;

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

#define HID_SERVICE_NVM_MEMORY_WORDS                      (2)

/* The offset of data being stored in NVM for HID service. This offset is added
 * to HID service offset to NVM region (see hid_data.nvm_offset) to get the 
 * absolute offset at which this data is stored in NVM
 */
#define HID_NVM_INPUT_BOOT_RPT_CLIENT_CONFIG_OFFSET       (0)
#define HID_NVM_INPUT_RPT_CLIENT_CONFIG_OFFSET            (1)

/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void updateInputReportConfiguration(gatt_client_config config, 
                                                                uint16 handle);
static void handleControlPointUpdate(hid_control_point_op control_op);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      updateInputReportConfiguration
 *
 *  DESCRIPTION
 *      This function is used to update client configurations.
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/

static void updateInputReportConfiguration(gatt_client_config config, 
                                                                 uint16 handle)
{
    /* Whenever CCCD of input report or boot input report are written, the
     * input report handle and notification status of input report need to be
     * updated as single variable is being maintained for both boot and report
     * protocol modes.
     */
    if(config & gatt_client_config_notification)
    {
        hid_data.input_report_handle = handle;
        hid_data.notify_enable = TRUE;
    }
    else
    {
        hid_data.notify_enable = FALSE;
    }
}    

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleControlPointUpdate
 *
 *  DESCRIPTION
 *      This function is used to handle HID Control Point updates
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/

static void handleControlPointUpdate(hid_control_point_op control_op)
{

    switch(control_op)
    {
        case hid_suspend:
        {
             hid_data.suspended = TRUE;

             /* Host has suspended its operations, application may like 
              * to do a low frequency key scan. The sample application is 
              * not doing any thing special in this case other than not sending
              * a connection parameter update request if the remote host is
              * currently suspended.
              */
        }
        break;

        case hid_exit_suspend:
        {
             hid_data.suspended = FALSE;

             /* Host has exited suspended mode, application may like 
              * to do a normal frequency key scan. The sample application is 
              * not doing any thing special in this case.
              */
        }
        break;

        default:
        {
            /* Ignore invalid value */
        }
        break;
    }

}

/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidDataInit
 *
 *  DESCRIPTION
 *      This function is used to initialise HID service data 
 *      structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HidDataInit(void)
{
    hid_data.input_report_handle = INVALID_ATT_HANDLE;
    hid_data.notify_enable = FALSE;

    if(!AppIsDeviceBonded())
    {
        /* Initialise Interrupt and Control Characteristic Client Configuration 
         * only if device is not bonded
         */
        hid_data.input_client_config = gatt_client_config_none;
        hid_data.input_boot_client_config = gatt_client_config_none;

        /* Update the NVM  with the same. */
        Nvm_Write((uint16*)&hid_data.input_boot_client_config,
                  sizeof(gatt_client_config), 
             hid_data.nvm_offset + HID_NVM_INPUT_BOOT_RPT_CLIENT_CONFIG_OFFSET);
        Nvm_Write((uint16*)&hid_data.input_client_config,
                  sizeof(gatt_client_config),
             hid_data.nvm_offset + HID_NVM_INPUT_RPT_CLIENT_CONFIG_OFFSET);
    }
    else /* Bonded */
    {
        /* Bonded and Notifications are configured for Input Reports */

        /* At every new connection, HID device will be reset to Report 
         * Mode (i.e., the default mode). So, check if client 
         * configuration for HID Input Report is already set, then move 
         * the device to NOTIFY state
         */

        if(hid_data.input_client_config & gatt_client_config_notification)
        {
            hid_data.input_report_handle = HANDLE_HID_INPUT_REPORT;
            hid_data.notify_enable = TRUE;
        }
    }

    /* Default to Report Mode */
    hid_data.report_mode = hid_report_mode;
    hid_data.suspended = FALSE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidInitChipReset
 *
 *  DESCRIPTION
 *      This function is used to initialise HID service data 
 *      structure at chip reset
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HidInitChipReset(void)
{
    /* HID service specific things that need to be done upon a power reset. */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidHandleAccessRead
 *
 *  DESCRIPTION
 *      This function handles Read operation on HID service attributes
 *      maintained by the application and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HidHandleAccessRead(GATT_ACCESS_IND_T *p_ind)
{
    uint16 length = 0;
    uint8  *p_value = NULL;
    uint8  val[2];
    sys_status rc = sys_status_success;


    switch(p_ind->handle)
    {

        case HANDLE_HID_INPUT_RPT_CLIENT_CONFIG:
        {
            p_value = val;

            BufWriteUint16(&p_value, hid_data.input_client_config);
            /* CCCD value of a characteristic is 2 bytes in length. */
            length = 2;

            /* BufWriteUint16 will have incremented p_value. Revert it back
             * to point to val.
             */
            p_value -= length;
        }
        break;

        case HANDLE_HID_BOOT_INPUT_RPT_CLIENT_CONFIG:
        {
            p_value = val;

            BufWriteUint16(&p_value, hid_data.input_boot_client_config);
            /* CCCD value of a characteristic is 2 bytes in length. */
            length = 2;

            /* BufWriteUint16 will have incremented p_value. Revert it back
             * to point to val.
             */
            p_value -= length;
        }
        break;

        case HANDLE_HID_INPUT_REPORT:
        {
            /* The input report array is maintained by mouse_hw.c. Get a
             * reference to it.
             */
            p_value = GetMouseReport(HID_INPUT_REPORT_ID);
            length = ATTR_LEN_HID_INPUT_REPORT;
        }
        break;

        case HANDLE_HID_BOOT_INPUT_REPORT:
        {
            /* The input report array is maintained by mouse_hw.c. Get a
             * reference to it.
             */
            p_value = GetMouseReport(HID_INPUT_REPORT_ID);
            length = ATTR_LEN_HID_BOOT_INPUT_REPORT;
        }
        break;

        case HANDLE_HID_PROTOCOL_MODE:
        {
            p_value = val;

            /* Get the current protocol mode. */
            p_value[0] = hid_data.report_mode;
            length = ATTR_LEN_HID_PROTOCOL_MODE;
        }
        break;

        case HANDLE_HID_FEATURE_REPORT:
        {
            p_value = val;

            val[0] = GetSensorResolution();
            
            /* Feature report value is 1 byte in length */
            length = ATTR_LEN_HID_FEATURE_REPORT;
        }
        break;

        default:
        {
            /* Let firmware handle the request */
            rc = gatt_status_irq_proceed;
        }
        break;
    }

    GattAccessRsp(p_ind->cid, p_ind->handle, rc,
                  length, p_value);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidHandleAccessWrite
 *
 *  DESCRIPTION
 *      This function handles Write operation on HID service attributes 
 *      maintained by the application.and responds with the GATT_ACCESS_RSP 
 *      message.
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/

extern void HidHandleAccessWrite(GATT_ACCESS_IND_T *p_ind)
{
    uint16 client_config, input_rpt_hndl;
    uint8 *p_value = p_ind->value;
    sys_status rc = sys_status_success;
    bool update_input_report_config = FALSE;

    switch(p_ind->handle)
    {
        case HANDLE_HID_INPUT_RPT_CLIENT_CONFIG:
        case HANDLE_HID_BOOT_INPUT_RPT_CLIENT_CONFIG:
        {
            /* Copy the value being written by the remote device to a local
             * variable
             */
            client_config = BufReadUint16(&p_value);


            /* Client Configuration is bit field value so ideally bitwise 
             * comparison should be used but since the application supports only 
             * notifications, direct comparison is being used.
             */
            if((client_config == gatt_client_config_notification) ||
               (client_config == gatt_client_config_none))
            {
                /* CCCD of input report and boot input report have different
                 * offsets at which their values are written into NVM
                 */
                uint16 offset;

                if(p_ind->handle == HANDLE_HID_INPUT_RPT_CLIENT_CONFIG)
                {
                    hid_data.input_client_config = client_config;

                    /* If Report mode set, update Input Report Configuration 
                     * for Report mode
                     */
                    if(hid_data.report_mode)
                    {
                        update_input_report_config = TRUE;
                        input_rpt_hndl = HANDLE_HID_INPUT_REPORT;
                    }

                    offset =  hid_data.nvm_offset+ 
                              HID_NVM_INPUT_RPT_CLIENT_CONFIG_OFFSET;

                }
                else /* Input Boot Report Client Configuration */
                {
                    hid_data.input_boot_client_config = client_config;

                    /* If Report mode not set, update Input Report Configuration 
                     * for Boot mode
                     */
                    if(!hid_data.report_mode)
                    {
                        update_input_report_config = TRUE;
                        input_rpt_hndl = HANDLE_HID_BOOT_INPUT_REPORT;
                    }

                    offset =  hid_data.nvm_offset+ 
                              HID_NVM_INPUT_BOOT_RPT_CLIENT_CONFIG_OFFSET;

                }

                /* Write HID Input Report Client configuration or HID Input Boot 
                 * Report Client configuration to NVM if the devices are bonded.
                 */
                 Nvm_Write(&client_config,
                          sizeof(gatt_client_config),
                          offset);

                /* Device shall only trigger Input reports (boot / main) if 
                 * notifications are enabled for characteristic corresponding
                 * to the current protocol mode.
                 */
                if(update_input_report_config)
                {
                    updateInputReportConfiguration(client_config, 
                                                   input_rpt_hndl);
                }
            }
            else
            {
                /* INDICATION or RESERVED */

                /* Return Error as only Notifications are supported for 
                 * mouse application
                 */

                rc = gatt_status_desc_improper_config;
            }
        }
        break;

        case HANDLE_HID_CONTROL_POINT:
        {
            /* Copy the value being written by the remote device to a local
             * variable.
             */
            uint8 control_op = BufReadUint8(&p_value);

            handleControlPointUpdate(control_op);
        }
        break;

        case HANDLE_HID_PROTOCOL_MODE:
        {
            /* Copy the value being written by the remote device to a local
             * variable.
             */
            uint8 mode = BufReadUint8(&p_value);
            /* Update the protocol mode only if the value being written is not
             * the same as the already existing protocol mode.
             */
            hid_protocol_mode old_mode = hid_data.report_mode;

            if(((mode == hid_boot_mode) ||
                (mode == hid_report_mode)) &&
                (mode != old_mode)) /* Change of Protocol Mode */
            {
                if(mode == hid_boot_mode)
                {
                    client_config = hid_data.input_boot_client_config;
                    input_rpt_hndl = HANDLE_HID_BOOT_INPUT_REPORT;
                }
                else
                {
                    client_config = hid_data.input_client_config;
                    input_rpt_hndl = HANDLE_HID_INPUT_REPORT;
                }

                hid_data.report_mode = mode;

                /* After the protocol mode is updated, update the service
                 * notification status from the notifications
                 * on CCCD of the respective mode handles.
                 */
                updateInputReportConfiguration(client_config,
                                               input_rpt_hndl);

            } /* Else Ignore the value */
        }
        break;

        case HANDLE_HID_FEATURE_REPORT:
        {
            /* Copy the value being written by the remote device to a local
             * variable.
             */
            sensor_resolution resolution = BufReadUint8(&p_value);

            /* Update the mouse resolution. This is an example feature report.
             * Customers can define vendor specific feature reports and
             * implement their handling as per the requirements
             */
            if(!SetSensorResolution(resolution))
            {
                rc = gatt_status_att_val_oor;
            }
        }
        break;
        
        default:
        {
            /* No more IRQ characteristics in HID service */
            rc = gatt_status_write_not_permitted;
        }
        break;
    }

    /* Send ACCESS RESPONSE */
    GattAccessRsp(p_ind->cid, p_ind->handle, rc, 0, NULL);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidIsNotifyEnabledOnReportId
 *
 *  DESCRIPTION
 *      This function returns whether notifications are enabled on CCCD of
 *      reports specified by 'report_id'.
 *
 *  RETURNS
 *      TRUE/FALSE: Notification configured or not
 *
 *----------------------------------------------------------------------------*/

extern bool HidIsNotifyEnabledOnReportId(uint8 report_id)
{
    switch(report_id)
    {
        case HID_INPUT_REPORT_ID: /* whether notifications are enabled on boot
                                   *  input or input report handle.
                                   */
            return hid_data.notify_enable;

        default:
            return FALSE;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GetReportMode
 *
 *  DESCRIPTION
 *      This function returns the present protocol mode the mouse is operating
 *      in.
 *
 *  RETURNS
 *      Present protocol mode.
 *
 *---------------------------------------------------------------------------*/

extern hid_protocol_mode GetReportMode(void)
{
    return hid_data.report_mode;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidSendInputReport
 *
 *  DESCRIPTION
 *      This function is used to notify mouse data to connected host
 *
 *----------------------------------------------------------------------------*/

extern void HidSendInputReport(uint16 ucid, uint8 report_id, uint8 *report)
{
    switch(report_id)
    {
        case HID_INPUT_REPORT_ID:
        {
            uint16 data_length = ATTR_LEN_HID_INPUT_REPORT;

            /* If boot mode is enabled, change the length of the report */
            if(!hid_data.report_mode)
            {
                data_length = ATTR_LEN_HID_BOOT_INPUT_REPORT;
            }
            GattCharValueNotification(ucid,
                              hid_data.input_report_handle,
                              data_length,
                              report);
        }
        break;
    
        /* More cases can be added if the mouse supports more types of
         * reports.
         */
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidReadDataFromNVM
 *
 *  DESCRIPTION
 *      This function is used to read HID service specific data store in NVM
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HidReadDataFromNVM(bool bonded, uint16 *p_offset)
{
    hid_data.nvm_offset = *p_offset;

    /* Read NVM only if devices are bonded */
    if(bonded)
    {
        /* Read HID Boot Input Report Client Configuration */
        Nvm_Read((uint16*)&hid_data.input_boot_client_config,
                   sizeof(gatt_client_config),
                   *p_offset + HID_NVM_INPUT_BOOT_RPT_CLIENT_CONFIG_OFFSET);

        /* Read HID Input Report Client Configuration */
        Nvm_Read((uint16*)&hid_data.input_client_config,
                   sizeof(gatt_client_config),
                   *p_offset + HID_NVM_INPUT_RPT_CLIENT_CONFIG_OFFSET);
    }
    
    /* Increment the offset by the number of words of NVM memory required 
     * by HID service 
     */
    *p_offset += HID_SERVICE_NVM_MEMORY_WORDS;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidCheckHandleRange
 *
 *  DESCRIPTION
 *      This function is used to check if the handle belongs to the HID 
 *      service
 *
 *  RETURNS
 *      Boolean - Indicating whether handle falls in range or not.
 *
 *----------------------------------------------------------------------------*/

extern bool HidCheckHandleRange(uint16 handle)
{
    return ((handle >= HANDLE_HID_SERVICE) &&
            (handle <= HANDLE_HID_SERVICE_END))
            ? TRUE : FALSE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HidIsStateSuspended
 *
 *  DESCRIPTION
 *      This function is used to check if the HID host has entered suspend state
 *
 *  RETURNS
 *      Boolean - TRUE if the remote device is in suspended state.
 *
 *----------------------------------------------------------------------------*/
 extern bool HidIsStateSuspended(void)
{
    return hid_data.suspended;
}


#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteHIDServiceDataInNvm
 *
 *  DESCRIPTION
 *      This function writes HID service data in NVM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void WriteHIDServiceDataInNvm(void)
{
    /* Write HID Boot Input Report Client Configuration */
    Nvm_Write((uint16*)&hid_data.input_boot_client_config,
               sizeof(gatt_client_config),
         hid_data.nvm_offset + HID_NVM_INPUT_BOOT_RPT_CLIENT_CONFIG_OFFSET);
    
    /* Write HID Input Report Client Configuration */
    Nvm_Write((uint16*)&hid_data.input_client_config,
               sizeof(gatt_client_config),
              hid_data.nvm_offset + HID_NVM_INPUT_RPT_CLIENT_CONFIG_OFFSET);
}
#endif /* NVM_TYPE_FLASH */

