/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      battery_service.h
 *
 *  DESCRIPTION
 *      Header definitions for Battery service
 *
 ******************************************************************************/

#ifndef _BATT_SERVICE_H_
#define _BATT_SERVICE_H_

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <types.h>
#include <bt_event_types.h>

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to initialise battery service data structure.*/
extern void BatteryDataInit(void);

/* This function is used to initialise battery service data structure at 
 * chip reset
 */
extern void BatteryInitChipReset(void);

/* This function handles read operation on battery service attributes
 * maintained by the application
 */
extern void BatteryHandleAccessRead(GATT_ACCESS_IND_T *p_ind);

/* This function handles write operation on battery service attributes 
 * maintained by the application
 */
extern void BatteryHandleAccessWrite(GATT_ACCESS_IND_T *p_ind);

/* This function is to monitor the battery level and trigger notifications
 * (if configured) to the connected host
 */
extern void BatteryUpdateLevel(uint16 ucid);

/* This function is used to read battery service specific data stored in 
 * NVM
 */
extern void BatteryReadDataFromNVM(bool bonded, uint16 *p_offset);

/* This function is used to check if the handle belongs to the Battery 
 * service
 */
extern bool BatteryCheckHandleRange(uint16 handle);

#ifdef NVM_TYPE_FLASH
/* This function writes Battery service data in NVM */
extern void WriteBatteryServiceDataInNvm(void);
#endif /* NVM_TYPE_FLASH */

#endif /* _BATT_SERVICE_H_ */
