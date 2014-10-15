/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      hid_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for HID service
 *
 ******************************************************************************/

#ifndef _HID_UUIDS_H_
#define _HID_UUIDS_H_

/*=============================================================================*
 *         Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser 
 * which creates .c and .h files from .db file doesn't understand
 * brackets and will raise syntax errors.
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/Pages/
 * ServiceViewer.aspx?u=org.bluetooth.service.generic_access.xml.
 */

/* HID service UUID */
#define HID_SERVICE_UUID                                                  0x1812

/* Information characteristic UUID */
#define HID_INFORMATION_UUID                                              0x2a4a

/* Report map(report descriptor) characteristic UUID */
#define HID_REPORT_MAP_UUID                                               0x2a4b

/* HID control point characteristic UUID */
#define HID_CONTROL_POINT_UUID                                            0x2a4c

/* HID report characteristic UUID. The same UUID will be used to declare input,
 * output and feature reports
 */
#define HID_REPORT_UUID                                                   0x2a4d

/* HID external report reference UUID. Its used when HID report gives
 * information pertaining to another service such as Battery service
 */
#define HID_EXT_REPORT_REFERENCE_UUID                                     0x2907

/* HID report reference UUID. Used to declare the report ID and report types of
 * different types of reports
 */
#define HID_REPORT_REFERENCE_UUID                                         0x2908

/* HID protocol mode characteristic UUID. Boot or report protocol mode will be
 * used during HID operations
 */
#define HID_PROTOCOL_MODE_UUID                                            0x2a4e

/* Characteristic UUID for the input report used by keyboard in boot mode */
#define HID_BOOT_KEYBOARD_INPUT_REPORT_UUID                               0x2a22

/* Characteristic UUID for the output report used by keyboard in boot mode */
#define HID_BOOT_KEYBOARD_OUTPUT_REPORT_UUID                              0x2a32

/* Characteristic UUID for the input report used by mouse in boot mode */
#define HID_BOOT_MOUSE_INPUT_REPORT_UUID                                  0x2a33

#endif /* _HID_UUIDS_H_ */