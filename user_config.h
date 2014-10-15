/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      user_config.h
 *
 *  DESCRIPTION
 *      This file contains definitions which will enable customization of the
 *      application.
 *
 ******************************************************************************/

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Un-comment below macro to enable GAP Peripheral Privacy support */
/* If Privacy is supported, the mouse uses resolvable random address for
 * advertising. Peripheral privacy flag and re-connection address
 * characteristics are supported when privacy is supported.
 */
 /*#define __GAP_PRIVACY_SUPPORT__ */

/* Uncomment the below macro if the mouse should never disconnect from the host
 * it is connected to.
 */
/* #define _NO_IDLE_TIMEOUT_ */

/* The number of buttons on the mouse */
#define N_MOUSE_BUTTONS                 3

/* This macro holds the report ID of the input report sent to transmit mouse
 * movement data. In the present descriptor(in hid_service_db.db), the report ID
 * of this report is not present. Hence, this is assigned to 0 in report
 * reference descriptor of HID_INPUT_REPORT characteristic
 * (also in hid_service_db.db). But when more report types are added to the
 * report descriptor, this report will take a report ID of 1. So, for the 
 * communication between application and hid_service.c and .h files, this
 * report ID is being kept as 1.
 */
#define HID_INPUT_REPORT_ID             1

/* A mouse can support different kinds of reports(with different report IDs).
 * This macro defines the length of the longest of all the reports supported
 * by the mouse. In the current implementation, the largest report is of size
 * 6. This macro will have to be modified according to any changes in the report
 * descriptor in hid_service_db.db.
 */
#define ATTR_LEN_LARGEST_REPORT         6

/* Parser version (UINT16) - Version number of the base USB HID specification
 * Format - 0xJJMN (JJ - Major Version Number, M - Minor Version 
 *                  Number and N - Sub-minor version number)
 */
#define HID_FLAG_CLASS_SPEC_RELEASE     0x0213

/* Country Code (UINT8)- Identifies the country for which the hardware is 
 * localized. If the hardware is not localized the value shall be Zero.
 */
#define HID_FLAG_COUNTRY_CODE           0x00

/* Bitfields to be used in HID_INFO_FLAGS to indicate the features supported
 * by the mouse.
 */
#define REMOTE_WAKEUP_SUPPORTED         0x01

/* If this is supported, then the mouse will have to keep advertising
 * continuously when it's not connected. This will need changes in the
 * application. This will also drain out the battery very soon. So, it is not
 * advisable to have this feature supported.
 */
#define NORMALLY_CONNECTABLE            0x02

/* If the application supports normally connectable feature, bitwise OR the
 * HID_INFO_FLAGS value with NORMALLY_CONNECTABLE
 */
#define HID_INFO_FLAGS                  REMOTE_WAKEUP_SUPPORTED

/* Values of different fields of PNP_ID characteristic of device infomation
 * service.
 */

#define VENDOR_ID                       0x000A
#define PRODUCT_ID                      0x014D
#define PRODUCT_VERSION                 0x0100

#endif /* _USER_CONFIG_H_ */
