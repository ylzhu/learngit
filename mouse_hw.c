/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      mouse_hw.c
 *
 *  DESCRIPTION
 *      This file defines the mouse hardware specific routines.
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <pio.h>
#include <pio_ctrlr.h>
#include <sleep.h>
#include <mem.h>
#include <timer.h>

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "app_gatt.h"
#include "mouse.h"
#include "mouse_hw.h"
#include "sensor.h"
#include "user_config.h"

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Mouse buttons PIO assignments */
#define BUTTON_LEFT_PIO                 (5)             /* left button */
#define BUTTON_MIDDLE_PIO               (6)             /* middle button */
#define BUTTON_RIGHT_PIO                (7)             /* middle button */
#define BUTTON_PAIRING_PIO              (8)             /* pairing button */

/* button PIO masks */
#define BUTTON_LEFT_PIO_MASK            (0x01UL << BUTTON_LEFT_PIO)
#define BUTTON_MIDDLE_PIO_MASK          (0x01UL << BUTTON_MIDDLE_PIO)
#define BUTTON_RIGHT_PIO_MASK           (0x01UL << BUTTON_RIGHT_PIO)
#define BUTTON_PAIRING_PIO_MASK         (0x01UL << BUTTON_PAIRING_PIO)

#define BUTTON_PIO_MASK       (BUTTON_LEFT_PIO_MASK | BUTTON_MIDDLE_PIO_MASK |\
                               BUTTON_RIGHT_PIO_MASK)

/* Mouse wheel quadrature decoder PIO assignments */
#define WHEEL_ZA_PIO                    (10)
#define WHEEL_ZB_PIO                    (11)

/* wheel PIO masks */
#define WHEEL_ZA_PIO_MASK               (0x01UL << WHEEL_ZA_PIO)
#define WHEEL_ZB_PIO_MASK               (0x01UL << WHEEL_ZB_PIO)

#define WHEEL_PIO_MASK                  (WHEEL_ZA_PIO_MASK | WHEEL_ZB_PIO_MASK)

/* Wheel base value that indicates the zero, no-movement wheel state*/
#define WHEEL_BASE_VALUE             (0x7F)

/* LED definitions */
/* The low power PIO is not being used now. In future, this can be used upon
 * receiving sys_event_battery_low system event
 */
#define LED_LOW_POWER_PIO               (9)
#define LED_LOW_POWER_PIO_MASK          (0x01UL << LED_LOW_POWER_PIO)

#define LED_PAIR_PIO                    (2)
#define LED_PAIR_PIO_MASK               (0x01UL << LED_PAIR_PIO)

/* PIO direction */
#define PIO_DIRECTION_INPUT             (FALSE)
#define PIO_DIRECTION_OUTPUT            (TRUE)

/* De-bouncing timer for the pairing removal button */
#define PAIRING_BUTTON_DEBOUNCE_TIME    (100 * MILLISECOND)

/* De-bouncing timer for the mouse buttons(left, right and middle) */
#define MOUSE_BUTTONS_DEBOUNCE_TIME     (5 * MILLISECOND)

/* The time for which the pairing removal button needs to be pressed
 * to remove pairing
 */
#define PAIRING_REMOVAL_TIMEOUT         (1 * SECOND)

/* PIO controller interface */
#define READ_WHEEL_VALUE()              (WHEEL_BASE_VALUE - ((*PIO_CONTROLLER_DATA_WORD) & 0x00FF))

#define RESET_WHEEL_VALUE()             (*(PIO_CONTROLLER_DATA_WORD + 2)) = 1

#define ENABLE_WHEEL_INTERRUPT(en)      *(PIO_CONTROLLER_DATA_WORD + 1) = en

/* Use PWM0 to control the Low power LED */
#define LOW_POWER_LED_PWM_INDEX         (0)

/* PWM0 Configuration Parameters */
/* DULL ON Time - 0 * 30us = 0us */
#define LOW_POWER_LED_DULL_ON_TIME      (0)

/* DULL OFF Time - 200 * 30us =  6 ms */
#define LOW_POWER_LED_DULL_OFF_TIME     (200)

/* DULL HOLD Time - 124 * 16ms =  1984 ms */
#define LOW_POWER_LED_DULL_HOLD_TIME    (124)

/* BRIGHT OFF Time - 0 * 30us = 0us */
#define LOW_POWER_LED_BRIGHT_OFF_TIME   (0)

/* BRIGHT ON Time - 1 * 30us = 30us */
#define LOW_POWER_LED_BRIGHT_ON_TIME    (1)

/* BRIGHT HOLD Time - 1 * 16ms = 16ms */
#define LOW_POWER_LED_BRIGHT_HOLD_TIME  (1)

/* RAMP RATE - 0 * 30us = 0us */
#define LOW_POWER_LED_RAMP_RATE         (0)

/*=============================================================================*
 *  Private Data
 *============================================================================*/

static MOUSE_REPORTS_T mouse_report;

/* Timer to hold the debouncing timer for the pairing removal button press.
 * The same timer is used to hold the debouncing timer for the pairing
 * button
 */
timer_id pairing_removal_tid;

/* The status of the mouse buttons after debouncing logic is applied */
uint8 mouse_button_status;

/* The array to hold the masks for the buttons on mouse */
uint32 button_masks[N_MOUSE_BUTTONS] = {
                                        BUTTON_LEFT_PIO_MASK,
                                        BUTTON_RIGHT_PIO_MASK,
                                        BUTTON_MIDDLE_PIO_MASK
                                       };

/* Array to hold the debounce timers for all the buttons on mouse */
timer_id button_debounce_tids[N_MOUSE_BUTTONS];

/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* Quadrature decoder implementation */
void quad_decoder(void);
static void handlePairPioStatusChange(timer_id tid);
static void handleMouseButtonStatusChange(timer_id tid);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handlePairPioStatusChange
 *
 *  DESCRIPTION
 *      This function is called upon detecting a pairing button press. On the
 *      actual hardware this may be seen as a 'connect' button.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handlePairPioStatusChange(timer_id tid)
{
    if(tid == pairing_removal_tid)
    {
        
        /* Get the PIOs status to check whether the pairing button is pressed or
         * released
         */
        uint32 pios = PioGets();

        /* Reset the pairing removal timer */
        pairing_removal_tid = TIMER_INVALID;

        /* If the pairing button is still pressed after the expiry of debounce
         * time, then create a timer of PAIRING_REMOVAL_TIMEOUT after which
         * pairing will be removed. Before the expiry of this timer, if a
         * pairing button removal is detected, then this timer will be deleted
         */
        if((!(pios & BUTTON_PAIRING_PIO_MASK)))
        {
            /* Create a timer after the expiry of which pairing information
             * will be removed
             */
            pairing_removal_tid = TimerCreate(PAIRING_REMOVAL_TIMEOUT, TRUE,
                                                      HandlePairingButtonPress);
        }

        /* Re-enable events on the pairing button */
        PioSetEventMask(BUTTON_PAIRING_PIO_MASK, pio_event_mode_both);

    } /* Else ignore the function call as it may be due to a race condition */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleMouseButtonStatusChange
 *
 *  DESCRIPTION
 *      This function is called upon the expiry of debounce timer after a mouse
 *      button PIO status change was detected
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handleMouseButtonStatusChange(timer_id tid)
{
    uint16 button = 0;
    uint32 pios = PioGets();
    
    for(button = 0; button < N_MOUSE_BUTTONS; button++)
    {
        if(tid == button_debounce_tids[button])
        {
            /* The timer is deleted by the firmware. So, reset the timer value
             * maintained by the application
             */
            button_debounce_tids[button] = TIMER_INVALID;

            /* Re-enable events on the button press/release */
            PioSetEventMask(button_masks[button], pio_event_mode_both);

            /* Check whether the button is pressed */
            if(!(pios & button_masks[button]))
            {
                /* Update the button status with the button that is pressed */
                mouse_button_status |= (1 << button);
            }

            else /* Button is released */
            {
                mouse_button_status &= (~(1 << button));
            }
        }
    }
    /* If the mouse is not in active state, then, it needs to be moved
     * to that state to send the mouse button press/release as reports
     */
    HandleMousePioStatusChange();
}

/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      SetUpPairLED
 *
 *  DESCRIPTION
 *      PIO2 usage has to be toggled between LED operation and driving the NVM.
 *      We need to be careful to make sure that LED operation and EEPROM
 *      operation don't happen at the same time. Normally PIO2 will be
 *      configured as PAIR_PIO. Whenever a NVM operation is to be done,configure
 *      PIO2 for driving the NVM operation. Once the NVM operation is done, this
 *      function is called to reconfigure PIO2 as LED.
 *
 *  RETURNS
 *      None
 *----------------------------------------------------------------------------*/

extern void SetUpPairLED(void)
{
    PioSetModes(LED_PAIR_PIO_MASK, pio_mode_user);
    PioSetDir(LED_PAIR_PIO, TRUE);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      InitHardware  -  intialise mouse scanning hardware
 *
 *  DESCRIPTION
 *      This function is called upon a power reset to initialize the PIOs
 *      and configure their initial states. It also puts the initial
 *      configuration for sensor in place.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void InitHardware(void)
{
    /* Set up the PIO controller */
    PioCtrlrInit((uint16*)&quad_decoder);

    /* Don't wakeup on UART RX line */
    SleepWakeOnUartRX(FALSE);

    /* setup mouse buttons */
    PioSetModes(BUTTON_PIO_MASK | BUTTON_PAIRING_PIO_MASK, pio_mode_user);
    PioSetPullModes(BUTTON_PIO_MASK | BUTTON_PAIRING_PIO_MASK, pio_mode_strong_pull_up);

    PioSetDirs(BUTTON_PIO_MASK | BUTTON_PAIRING_PIO_MASK, PIO_DIRECTION_INPUT);

    PioSetEventMask(BUTTON_PAIRING_PIO_MASK, pio_event_mode_both);

    /* assign mouse wheel PIOs to PIO controller */
    PioSetModes(WHEEL_PIO_MASK, pio_mode_pio_controller);
    PioSetPullModes(WHEEL_PIO_MASK, pio_mode_strong_pull_down);
    
    /* set up LEDs */

    /* Set up the low power LED to normal user mode. When the application wants
     * to glow it, it will be re-configured to be controlled through PWM0.
     */
    PioSetMode(LED_LOW_POWER_PIO, pio_mode_user);
    PioSetDir(LED_LOW_POWER_PIO, PIO_DIRECTION_OUTPUT);
    PioSet(LED_LOW_POWER_PIO, FALSE);
    
    SetUpPairLED();

    /* initialize the sensor */
    SensorInit(SENSOR_LASER_CURRENT_1, SENSOR_LASER_MINIMUM_POWER);

    /* configure sensor run and rest modes */
    SensorConfigureModes();
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GlowLowPowerLED  -  Enable/Disable the Low power LED mapped to PWM0.
 *
 *  DESCRIPTION
 *      This function is called to enable/disable the low power LED controlled 
 *      through PWM0.
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *----------------------------------------------------------------------------*/
extern void GlowLowPowerLED(bool enable)
{
    if(enable)
    {
        /* Setup Low power LED - PIO-9 controlled through PWM0.*/
        PioSetMode(LED_LOW_POWER_PIO, pio_mode_pwm0);
        
        /* Configure the PWM0 parameters */
        /* Configure the PWM0 parameters. */
        PioConfigPWM(LOW_POWER_LED_PWM_INDEX, pio_pwm_mode_push_pull,
                  LOW_POWER_LED_DULL_ON_TIME, LOW_POWER_LED_DULL_OFF_TIME,
                  LOW_POWER_LED_DULL_HOLD_TIME, LOW_POWER_LED_BRIGHT_ON_TIME,
                  LOW_POWER_LED_BRIGHT_OFF_TIME, LOW_POWER_LED_BRIGHT_HOLD_TIME,
                  LOW_POWER_LED_RAMP_RATE);

        /* Enable the PWM0 */
        PioEnablePWM(LOW_POWER_LED_PWM_INDEX, TRUE);
    }
    else
    {
        /* Disable the PWM0 */
        PioEnablePWM(LOW_POWER_LED_PWM_INDEX, FALSE);
        /* Reconfigure LED_LOW_POWER_PIO to pio_mode_user. This reconfiguration
         * has been done because when PWM is disabled, LED_LOW_POWER_PIO value
         * may remain the same as it was, at the exact time of disabling. So if
         * LED_LOW_POWER_PIO was on, it may remain ON even after disabling PWM.
         * So, it is better to reconfigure it to user mode. It will get 
         * reconfigured to PWM mode when we re-enable the LED.
         */
        PioSetMode(LED_LOW_POWER_PIO, pio_mode_user);
        PioSetDir(LED_LOW_POWER_PIO, PIO_DIRECTION_OUTPUT);
        PioSet(LED_LOW_POWER_PIO, FALSE);
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      buttonDataInit
 *
 *  DESCRIPTION
 *      This function is called to delete the debounce timers of mouse buttons
 *      and set the button status to 'all buttons released'.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
extern void ButtonDataInit(void)
{
    uint8 button;
    
    mouse_button_status = 0x00;

    for(button = 0; button < N_MOUSE_BUTTONS; button++)
    {
        /* Delete the debouncing timers for the mouse buttons and reset their
         * values
         */
        TimerDelete(button_debounce_tids[button]);
        
        button_debounce_tids[button] = TIMER_INVALID;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HwDataInit
 *
 *  DESCRIPTION
 *      This function initializes the mouse data structure handled by
 *      mouse_hw.c, that is, all the reports supported by mouse.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HwDataInit(void)
{
    /* Set all the reports supported by mouse to 0 */
    MemSet(mouse_report.input_report, 0, ATTR_LEN_HID_INPUT_REPORT);

    ButtonDataInit();

    ENABLE_WHEEL_INTERRUPT(FALSE);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      DisableMouseMotionAndWheelEvents
 *
 *  DESCRIPTION
 *      This function disables wake up event on mouse movement and any further
 *      events on scroll of a wheel
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void DisableMouseMotionAndWheelEvents(void)
{
    /* disable events on MOTION# signal */
    SleepWakePinEnable(wakepin_mode_disable);

    /* Disable events on wheel scroll */
    ENABLE_WHEEL_INTERRUPT(FALSE);
    
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      DisableMouseEvents
 *
 *  DESCRIPTION
 *      This function disables both:
 *      pio_events on the press of mouse buttons and
 *      wake up event on mouse movement.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void DisableMouseEvents(void)
{
    /* disable events on MOTION# signal */
    SleepWakePinEnable(wakepin_mode_disable);

    PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_disable);

    ButtonDataInit();

    ENABLE_WHEEL_INTERRUPT(FALSE);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      EnableMouseMotionDetection
 *
 *  DESCRIPTION
 *      This function enables the event 'sys_event_wake_up' to reach the
 *      application whenever the user moves the mouse.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void EnableMouseMotionDetection(void)
{
    /* If there is already some data in the mouse sensor registers, clear the
     * data, thereby setting back the wake pin status to high which enables
     * the application to get a system wake pin event when there is new data in
     * the mouse motion sensing registers.
     */
    if(!SleepWakePinStatus())
    {
        WriteToMotionReg();
    }
    
    /* Re-enable wake on MOTION# signal */
    SleepWakePinEnable(wakepin_mode_low_level);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      EnableMouseMotionAndButtonEvents
 *
 *  DESCRIPTION
 *      This function enables both:
 *      pio_events on the press of mouse buttons and
 *      wake up event on mouse movement.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void EnableMouseMotionAndButtonEvents(void)
{
    EnableMouseMotionDetection();
    
    /* re-enable button and wheel events */
    PioSetEventMask(BUTTON_PIO_MASK, pio_event_mode_both);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      ReEnableMouseEvents
 *
 *  DESCRIPTION
 *      This function enables:
 *      pio_events on the press of mouse buttons,
 *      wake up event on mouse movement,
 *      pio_cntrlr_changed events on wheel scroll.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void ReEnableMouseEvents(void)
{
    /* Enable the events that reach the application when the user moves the
     * mouse or presses one of the buttons
     */
    EnableMouseMotionAndButtonEvents();
    
    ENABLE_WHEEL_INTERRUPT(TRUE);

    /* reset wheel value */
    RESET_WHEEL_VALUE();
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      StartHardware
 *
 *  DESCRIPTION
 *      Sets up all PIOs and starts PIO controller for mouse operation
 *
 *  RETURNS
 *      None
 *----------------------------------------------------------------------------*/

extern void StartHardware(void)
{
    /* pull up left, right, middle button PIOs and also ZA and ZB */
    PioSetPullModes(WHEEL_PIO_MASK, pio_mode_strong_pull_up);
    
    /* start quadrature decoder code */
    PioCtrlrStart();
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      StopHardware
 *
 *  DESCRIPTION
 *      Reverts PIO events to defaults and stops PIO controller
 *
 *  RETURNS
 *      None
 *----------------------------------------------------------------------------*/

extern void StopHardware(void)
{

    ENABLE_WHEEL_INTERRUPT(FALSE);

    /* remove pull-ups on quadrature encoder inputs and mouse button PIOs */
    PioSetPullModes(WHEEL_PIO_MASK, pio_mode_strong_pull_down);

    /* stop quadrature decoder code */
    PioCtrlrStop();
    
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GetMouseData
 *
 *  DESCRIPTION
 *      Detects whether there is a new mouse report to be sent, if there is one,
 *      update 'mouse_reports'.
 *
 *  RETURNS
 *      TRUE if there is a new report to be sent.
 *
 *----------------------------------------------------------------------------*/

extern bool GetMouseData(uint8 hid_mode, uint8 *temp_input_report)
{
    uint8  status;
    bool   new_report = FALSE;
    uint16 dx = 0, dy = 0;
    uint16 report_length = ATTR_LEN_HID_INPUT_REPORT;

    /* If currently boot mode is set, change the mouse report length */
    if(!hid_mode)
    {
        report_length = ATTR_LEN_HID_BOOT_INPUT_REPORT;
    }    

    /* Initialize the data to be sent to zero */
    MemSet(temp_input_report, 0x00, report_length);
    
    /* Update the button status in the input report. */
    temp_input_report[0] = mouse_button_status;

    /* Compare it with the last sent button status. If it has changed, we need
     * to send a report.
     */
    if(temp_input_report[0] != mouse_report.input_report[0])
    {
        new_report = TRUE;
    }

    /* 6th byte of input report is the wheel value */
    temp_input_report[5] = READ_WHEEL_VALUE();

    /* Write the default value to the PIO controller shared memory so that
     * it can hold new data with reference to the wheel default value again.
     */
    RESET_WHEEL_VALUE();

    if(temp_input_report[5])
    {
        /* The wheel motion data is to be sent only if the present mode is
         * report protocol mode
         */     
        if(hid_mode)
        {
            new_report = TRUE;
        }
    }

    /* Read sensor data if WAKE/MOTION signal is low */
    if(!SleepWakePinStatus())
    {
        new_report = TRUE;
        
        /* Read the sensor values */
        SensorReadReport(&dx, &dy, &status);
        
        if(dy)
        {
            /* invert Y-axis */
            dy = (~dy & 0x0FFF) + 1;
        }

        /* Translate 12-bit signed 2's complement to 16-bit */
        if(dx > 0x07FF)
        {
            dx |= 0xF000;
        }

        if(dy > 0x07FF)
        {
            dy |= 0xF000;
        }

        /* First and second bytes of mouse report are same in both boot and
         * protocol mode. Populate this part of the report.
         */

        /* Store lower byte of delta-x */
        temp_input_report[1] = (uint8) dx;

        /* In boot mode, the report format is button, delta-x and delta-y with
         * each field occupying 1 byte each
         */
        if(!hid_mode)
        {
            /* Store lower byte of delta-y */
            temp_input_report[2] = (uint8) dy;

            /* Handle overflow while converting 12-bit data to 8-bit data */
            
            /* Overflow in delta-x */
            if(((int)dx) < -128)
            {
                /* Setting delta-x to the minimum value that can be taken by
                 * uint8
                 */
                temp_input_report[1] = -128;
            }
            else if(((int)dx) > 127)
            {
                /* Setting delta-x to the maximum value that can be taken by
                 * uint8
                 */
                temp_input_report[1] = 127;
            }

            /* Overflow in delta-y */
            if(((int)dy) < -128)
            {
                /* Setting delta-y to the minimum value that can be taken by
                 * uint8
                 */
                temp_input_report[2] = -128;
            }
            else if(((int)dy) > 127)
            {
                /* Setting delta-y to the maximum value that can be taken by
                 * uint8
                 */
                temp_input_report[2] = 127;
            }
        }
        
        else /* Report protocol */
        {            
            /* Create HID report */
            temp_input_report[2] = (uint8) (dx >> 8);
            temp_input_report[3] = (uint8) dy;       /* low byte first */
            temp_input_report[4] = (uint8) (dy >> 8) ;
        }
    }

    /* If a new report is to be sent, copy the new report to the global variable
     * so that the application can send the data.
     */
    if(new_report)
    {
        MemCopy(mouse_report.input_report, temp_input_report, report_length);
    }
    
    return new_report;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      GetMouseReport
 *
 *  DESCRIPTION
 *      This function is used to get a mouse report referred by the report ID
 *
 *  RETURNS
 *      Returns a pointer to the mouse report referred by the report ID
 *
 *----------------------------------------------------------------------------*/
extern uint8 *GetMouseReport(uint8 report_id)
{
    uint8 *report_ref = NULL;

    /* Based on the report ID, return the pointer to the specific report */
    switch(report_id)
    {
        case HID_INPUT_REPORT_ID:
        {
            report_ref = mouse_report.input_report;
        }
        break;

        default:
        break;
    }
    
    return report_ref;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      HandlePIOChangedEvent
 *
 *  DESCRIPTION
 *      This function handles PIO Changed event
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void HandlePIOChangedEvent(uint32 pio_changed)
{
    if(pio_changed & BUTTON_PAIRING_PIO_MASK)
    {
        /* Get the PIOs status to check whether the pairing button is
         * pressed or released
         */
        uint32 pios = PioGets();

        /* Delete the presently running timer */
        TimerDelete(pairing_removal_tid);

        /* If the pairing button is pressed....*/
        if(!(pios & BUTTON_PAIRING_PIO_MASK))
        {

            /* Disable any further events due to change in status of
             * pairing button PIO. These events are re-enabled once the
             * debouncing timer expires
             */
            PioSetEventMask(BUTTON_PAIRING_PIO_MASK, pio_event_mode_disable);

            pairing_removal_tid = TimerCreate(PAIRING_BUTTON_DEBOUNCE_TIME,
                                               TRUE, handlePairPioStatusChange);
        }

        else /* It's a pairing button release */
        {
            pairing_removal_tid = TIMER_INVALID;
        }                
    }
    /* A mouse button press/release is received */
    else
    {
        uint16 button;

        for (button = 0; button < N_MOUSE_BUTTONS; button++)
        {
            if (pio_changed & button_masks[button])
            {
                /* Disable any further events on this mouse button for the
                 * period of debounce time
                 */
                PioSetEventMask(button_masks[button], pio_event_mode_disable);

                /* Delete the presently running debounce timer */
                TimerDelete(button_debounce_tids[button]);

                /* Create a timer to handle debouncing of mouse buttons */
                button_debounce_tids[button]
                    = TimerCreate(MOUSE_BUTTONS_DEBOUNCE_TIME, TRUE,
                                  handleMouseButtonStatusChange);
            }
        }
    }
}
