/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      dev_info_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for Device info service
 *
 ******************************************************************************/
 
#ifndef _DEVICE_INFO_UUIDS_H_
#define _DEVICE_INFO_UUIDS_H_

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser which
 * creates .c and .h files from .db file doesn't understand brackets and will
 * raise syntax errors.
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/Pages/
 * ServiceViewer.aspx?u=org.bluetooth.service.device_information.xml.
 */

/* Device Information service */
#define UUID_DEVICE_INFO_SERVICE                  0x180A

/* System ID UUID */
#define UUID_DEVICE_INFO_SYSTEM_ID                0x2A23
/* Model number UUID */
#define UUID_DEVICE_INFO_MODEL_NUMBER             0x2A24
/* Serial number UUID */
#define UUID_DEVICE_INFO_SERIAL_NUMBER            0x2A25
/* Hardware revision UUID */
#define UUID_DEVICE_INFO_HARDWARE_REVISION        0x2A27
/* Firmware revision UUID */
#define UUID_DEVICE_INFO_FIRMWARE_REVISION        0x2A26
/* Software revision UUID */
#define UUID_DEVICE_INFO_SOFTWARE_REVISION        0x2A28
/* Manufacturer name UUID */
#define UUID_DEVICE_INFO_MANUFACTURER_NAME        0x2A29
/* PnP ID UUID */
#define UUID_DEVICE_INFO_PNP_ID                   0x2A50

/* Vendor Id Source */
#define VENDOR_ID_SRC_BT                          0x01
#define VENDOR_ID_SRC_USB                         0x02

#if defined(CSR101x_A05)
#define HARDWARE_REVISION "CSR101x A05"
#elif defined(CSR100x)
#define HARDWARE_REVISION "CSR100x A04"
#else
#define HARDWARE_REVISION "Unknown"
#endif

#endif /* _DEVICE_INFO_UUIDS_H_ */  
