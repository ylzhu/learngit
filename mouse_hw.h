/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      mouse_hw.h
 *
 *  DESCRIPTION
 *      Header definitions for mouse hardware specific functions.
 *
 ******************************************************************************/

#ifndef _MOUSE_HW_H_
#define _MOUSE_HW_H_

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <timer.h>

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "app_gatt_db.h"

/*=============================================================================*
 *  Public Data Types
 *============================================================================*/

/* Structure to hold all the types of report supported by mouse */
typedef struct
{
    uint8 input_report[ATTR_LEN_HID_INPUT_REPORT];
    
}MOUSE_REPORTS_T;

/*============================================================================*
 *  Public Data Declarations
 *============================================================================*/

extern timer_id pairing_removal_tid;

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function initializes the PIOs used by mouse and configures their initial
 * states
 */
extern void InitHardware(void);

/* This function is called to enable/disable the low power LED controlled through
 * PWM0.
 */
extern void GlowLowPowerLED(bool enable);

/* This function is called to delete the debounce timers of mouse buttons and
 * set the button status to 'all buttons released'
 */
extern void ButtonDataInit(void);

/* This function initializes the mouse data structure handled by mouse_hw.c */
extern void HwDataInit(void);

/* This function disables wake up event on mouse movement */
extern void DisableMouseMotionAndWheelEvents(void);

/* This function disables events reaching the application upon mouse motion or
 * button press
 */
extern void DisableMouseEvents(void);

/* This function re-enables events upon mouse motion */
extern void EnableMouseMotionDetection(void);

/* Enable the events that reach the application when the user moves the mouse or
 * presses one of the buttons
 */
extern void EnableMouseMotionAndButtonEvents(void);

/* This function re-enables events upon mouse motion or button press */
extern void ReEnableMouseEvents(void);

/* This function sets up all PIOs and starts PIO controller for mouse operation */
extern void StartHardware(void);

/* This function reverts PIO events to defaults and stops PIO controller */
extern void StopHardware(void);

/* This function detects whether there is a new mouse report to be sent*/
extern bool GetMouseData(uint8 hid_mode, uint8 *temp_input_report);

/* This function handles PIO Changed event */
extern void HandlePIOChangedEvent(uint32 pio_changed);

#endif /* _MOUSE_HW_H_ */
