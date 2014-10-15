/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      gap_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for GAP service
 *
 ******************************************************************************/

#ifndef _GAP_UUIDS_H_
#define _GAP_UUIDS_H_

/*=============================================================================*
 *         Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser 
 * which creates .c and .h files from .db file doesn't understand  brackets 
 * and will raise syntax errors.
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/
 * Pages/ServiceViewer.aspx?u=org.bluetooth.service.generic_access.xml
 */

/* Gap Service UUID */
#define UUID_GAP_SERVICE                 0x1800

/* Device name characteristic UUID */
#define UUID_DEVICE_NAME                 0x2A00

/* Appearance characteristic UUID */
#define UUID_APPEARANCE                  0x2A01

/* Peripheral Privacy Flag characteristic UUID */
#define UUID_PERIPHERAL_PRIVACY_FLAG     0x2A02

/* Re-connection address characteristic UUID */
#define UUID_RECONNECTION_ADDRESS        0x2A03

/* Peripheral Preferred Connection Parameters */
#define UUID_PER_PREF_CONN_PARAMS        0x2A04

#endif /* _GAP_UUIDS_H_ */
