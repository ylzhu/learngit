/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      mouse_gatt.h
 *
 *  DESCRIPTION
 *      Header file for mouse GATT-related routines
 *
 ******************************************************************************/

#ifndef _MOUSE_GATT_H_
#define _MOUSE_GATT_H_

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <gap_types.h>
#include <bt_event_types.h>

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function checks if the address is resolvable random or not */
extern bool IsAddressResolvableRandom(TYPED_BD_ADDR_T *addr);

/* This function starts advertisements */
extern void GattStartAdverts(bool fast_connection, gap_mode_connect connect_mode);

/* This function starts advertisements using fast connection parameters */
extern void GattTriggerFastAdverts(void);

/* This function stops on-going advertisements */
extern void GattStopAdverts(void);

/* This function handles GATT_ACCESS_IND message for attributes maintained by
 * the application
 */
extern void GattHandleAccessInd(GATT_ACCESS_IND_T *p_ind);

/* This function checks if the address is non-resolvable random or not */
extern bool IsAddressNonResolvableRandom(TYPED_BD_ADDR_T *addr);

#endif /* _MOUSE_GATT_H_ */
