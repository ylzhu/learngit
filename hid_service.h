/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      hid_service.h
 *
 *  DESCRIPTION
 *      Header definitions for HID service
 *
 ******************************************************************************/

#ifndef _HID_SERVICE_H_
#define _HID_SERVICE_H_

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "user_config.h"

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bt_event_types.h>

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Control operation values can be referred from http://developer.bluetooth.org/
 * gatt/characteristics/Pages/
 * CharacteristicViewer.aspx?u=org.bluetooth.characteristic.hid_control_point.xml
 */
/* Suspend command is sent from the host when it wants to enter power saving
 * mode. Exit suspend is used by host to resume normal operations.
 */
typedef enum
{
    hid_suspend      = 0,
    hid_exit_suspend = 1,
    hid_rfu

} hid_control_point_op;

/* Protocol mode values can be referred from http://developer.bluetooth.org/
 * gatt/characteristics/Pages/
 * CharacteristicViewer.aspx?u=org.bluetooth.characteristic.protocol_mode.xml
 */
/* When the HID host switches between boot mode and report mode, the protocol
 * mode characteristic is written with these values.
 */
typedef enum
{
    hid_boot_mode   = 0,
    hid_report_mode = 1

} hid_protocol_mode;

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function initializes the data structure of HID service */
extern void HidDataInit(void);

/* This function initializes HID service data structure at chip reset */
extern void HidInitChipReset(void);

/* This function handles read operation on HID service attributes maintained by
 * the application
 */
extern void HidHandleAccessRead(GATT_ACCESS_IND_T *p_ind);

/* This function handles write operation on HID service attributes maintained by
 * the application
 */
extern void HidHandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* This function checks whether notification has been enabled on a
 * report characteristic referred by the 'report_id'
 */
extern bool HidIsNotifyEnabledOnReportId(uint8 report_id);

/* This function returns the present protocol mode the mouse is operating in */
extern hid_protocol_mode GetReportMode(void);

/* This function sends reports as notifications for the report characteristics
 * of HID service
 */
extern void HidSendInputReport(uint16 ucid, uint8 report_id, uint8 *report);

/* This function reads HID service specific data stored in NVM */
extern void HidReadDataFromNVM(bool bonded, uint16 *p_offset);

/* This function checks if the handle belongs to the HID service */
extern bool HidCheckHandleRange(uint16 handle);

/* This function checks if the HID host has entered suspend state */
extern bool HidIsStateSuspended(void);

#ifdef NVM_TYPE_FLASH
/* This function writes HID service data in NVM */
extern void WriteHIDServiceDataInNvm(void);
#endif /* NVM_TYPE_FLASH */

#endif /* _HID_SERVICE_H_ */
