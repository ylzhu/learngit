/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      battery_uuids.h
 *
 *  DESCRIPTION
 *      UUID MACROs for Battery service
 *
 ******************************************************************************/

#ifndef _BATTERY_UUIDS_H_
#define _BATTERY_UUIDS_H_

/*=============================================================================*
 *         Public Definitions
 *============================================================================*/

/* Brackets should not be used around the value of a macro. The parser which 
 * creates .c and .h files from .db file doesn't understand brackets and will
 * raise syntax errors. 
 */

/* For UUID values, refer http://developer.bluetooth.org/gatt/services/Pages/
 * ServiceViewer.aspx?u=org.bluetooth.service.battery_service.xml.
 */
 
/* Battery Service UUID */
#define BATTERY_SERVICE_UUID          0x180f

/* Battery Level UUID */
#define BATTERY_LEVEL_UUID            0x2a19

#endif /* _BATTERY_UUIDS_H_ */
