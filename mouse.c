/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      mouse.c
 *
 *  DESCRIPTION
 *      This file defines a simple mouse application.
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <gatt.h>
#include <ls_app_if.h>
#include <pio.h>
#include <mem.h>
#include <security.h>
#include <pio_ctrlr.h>
#include <nvm.h>
#include <time.h>

/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "mouse.h"
#include "mouse_hw.h"
#include "app_gatt.h"
#include "gap_service.h"
#include "hid_service.h"
#include "battery_service.h"
#include "scan_param_service.h"
#include "app_gatt_db.h"
#include "mouse_gatt.h"
#include "sensor.h"
#include "nvm_access.h"
#include "gap_conn_params.h"

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

/******** TIMERS ********/

/* Maximum number of timers */
#define MAX_APP_TIMERS               (6 + N_MOUSE_BUTTONS)

/* Magic value to check the sanity of NVM region used by the application */
#define NVM_SANITY_MAGIC             (0xAB07)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD       (0)

/* NVM offset for bonded flag */
#define NVM_OFFSET_BONDED_FLAG       (NVM_OFFSET_SANITY_WORD + 1)

/* NVM offset for bonded device bluetooth address */
#define NVM_OFFSET_BONDED_ADDR       (NVM_OFFSET_BONDED_FLAG + \
                                      sizeof(app_data.bonded))

/* NVM offset for diversifier */
#define NVM_OFFSET_SM_DIV            (NVM_OFFSET_BONDED_ADDR + \
                                      sizeof(app_data.bonded_bd_addr))

/* NVM offset for IRK */
#define NVM_OFFSET_SM_IRK            (NVM_OFFSET_SM_DIV + \
                                      sizeof(app_data.diversifier))

/* Number of words of NVM used by application. Memory used by supported 
 * services is not taken into consideration here.
 */
#define N_APP_USED_NVM_WORDS         (NVM_OFFSET_SM_IRK + \
                                      MAX_WORDS_IRK)

/* Time after which a L2CAP connection parameter update request will be
 * re-sent upon failure of an earlier sent request.
 */
#define GAP_CONN_PARAM_TIMEOUT       (30 * SECOND)

/* TGAP(conn_pause_peripheral) defined in Core Specification Addendum 3 Revision
 * 2. A Peripheral device should not perform a Connection Parameter Update proc-
 * -edure within TGAP(conn_pause_peripheral) after establishing a connection.
 */
#define TGAP_CPP_PERIOD              (5 * SECOND)

/* TGAP(conn_pause_central) defined in Core Specification Addendum 3 Revision 2.
 * After the Peripheral device has no further pending actions to perform and the
 * Central device has not initiated any other actions within TGAP(conn_pause_ce-
 * -ntral), then the Peripheral device may perform a Connection Parameter Update
 * procedure.
 */
#define TGAP_CPC_PERIOD              (1 * SECOND)

/* Initialization value for app_data.sensor_poll_period. */
#define INVALID_SENSOR_POLL_PERIOD   0

/* The mouse application has to send one notification per connection interval.
 * The sensor poll timer is started upon receiving 'radio_event_first_tx'. The
 * application has to put data to the firmware queue 1.8 ms before the start
 * of the next connection interval. Once, the timer expires, the data is read
 * from the sensors and then sent to the firmware to be sent to the connected
 * device. Reading the sensor data takes 300 microseconds/0.3ms. So, the sensor
 * poll period should expire 2.1 ms before the start of the next connection
 * interval.
 *
 * For Robinson the time taken for the application to receive the 
 * 'radio_event_first_tx' has been observed to be 900 microseconds after the 
 * start of the connection event. To ensure that the sensor poll timer expires 
 * only once in a connection interval, the timer should start after the
 * application receives 'radio_event_first_tx' and expire 2.1ms before the next
 * connection event starts. The sum of these two periods gives 3000
 * microseconds. This period should be subtracted from the value of connection
 * interval period to get the value of the sensor poll timer to be used.
 *
 * For Baldrick, it's observed that the 'radio_event_first_tx' arrives at the
 * beginning of the connection interval and a value of 450 micro seconds has
 * been observed instead of 900 microseconds in case of Robinson. Hence, the
 * residual time has been calibrated for Baldrick with a value 2550 to save 
 * power.
 *
 * NOTE: The value of 900 micro seconds above has been found for the
 * case when the longest mouse report is of 6 bytes(48 bits) length.  If the
 * largest mouse report increases one bit, then this value has to be increased
 * by 1 micro second, that is, if the largest mouse report is of length 8 bytes,
 * then this value should be 3016.
 */
#if defined(CSR101x_A05)

#define RESIDUAL_TIME                2550

#elif defined(CSR100x)

#define RESIDUAL_TIME                3000

#endif
/*=============================================================================*
 *  Private Data
 *============================================================================*/

/* Mouse application data structure */
APP_DATA_T app_data;

/* Declare space for application timers. */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void readPersistentStore(void);
static void mouseDataInit(void);
static void appEnterIdleState(void);

#ifndef _NO_IDLE_TIMEOUT_

static void mouseIdleTimerHandler(timer_id tid);

#endif /* _NO_IDLE_TIMEOUT_ */

static void resetIdleTimer(void);
static void appEnterActiveState(void);
static void appSetState(app_state new_state);
static void appStartAdvert(void);
static void handleSignalGattAddDBCfm(void);
static void handleSignalGattCancelConnectCfm(void);
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T* event_data);
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *event_data);
static void handleSignalLmEncryptionChange(LM_EVENT_T *event_data);
static void handleSignalLmEvDisconnectComplete(
                               HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data);
static void handleSignalLmEvConnectionComplete(
                                       LM_EV_CONNECTION_COMPLETE_T *event_data);
static void handleSignalSmKeysInd(SM_KEYS_IND_T *event_data);
static void handleSignalSmPairingAuthInd(SM_PAIRING_AUTH_IND_T *p_event_data);
static void handleSignalSmDivApproveInd(SM_DIV_APPROVE_IND_T *p_event_data);
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *event_data);
static void handleSignalLsConnParamUpdateCfm(
                            LS_CONNECTION_PARAM_UPDATE_CFM_T *event_data);
static void handleSignalLsConnectionParamUpdateInd(LM_EVENT_T* event_data);
static void handleSignalLmConnectionUpdate(
                                         LM_EV_CONNECTION_UPDATE_T* event_data);
static void handleSignalLsRadioEventInd(void);
static void handleMouseMotionEvent(void);
static void handleBatteryLowEvent(battery_low_data *data);
static void handleGapCppTimerExpiry(timer_id tid);
static void requestConnParamUpdate(timer_id tid);
static bool sendMouseReports(void);
static void sensorPollTimerExpiryHandler(timer_id tid);
static void handleBondingChanceTimerExpiry(timer_id tid);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      This function is used to initialise and read NVM data
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void readPersistentStore(void)
{
    uint16 offset = N_APP_USED_NVM_WORDS;
    uint16 nvm_sanity = 0xffff;

    /* Read persistent storage to know if the device was last bonded 
     * to another device 
     */

    /* If the device was bonded, trigger advertisements for the bonded
     * host(using whitelist filter). If the device was not bonded, trigger
     * advertisements for any host to connect to the mouse.
     */

    Nvm_Read(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

    if(nvm_sanity == NVM_SANITY_MAGIC)
    {
        /* Read Bonded Flag from NVM */
        Nvm_Read((uint16*)&app_data.bonded, sizeof(app_data.bonded),
                                                        NVM_OFFSET_BONDED_FLAG);

        if(app_data.bonded)
        {

            /* Bonded Host Typed BD Address will only be stored if bonded flag
             * is set to TRUE. Read last bonded device address.
             */
            Nvm_Read((uint16*)&app_data.bonded_bd_addr, 
                               sizeof(TYPED_BD_ADDR_T), NVM_OFFSET_BONDED_ADDR);

            /* If device address is resovable then read the bonded device's IRK
             */
            if(IsAddressResolvableRandom(&app_data.bonded_bd_addr))
            {
                Nvm_Read(app_data.central_device_irk.irk, 
                                    MAX_WORDS_IRK,
                                    NVM_OFFSET_SM_IRK);
            }
        }

        else /* Case when we have only written NVM_SANITY_MAGIC to NVM but
             * didn't get bonded to any host in the last powered session
             */
        {
            app_data.bonded = FALSE;
        }

        
        /* Read the diversifier associated with the presently bonded/last bonded
         * device.
         */
        Nvm_Read(&app_data.diversifier, sizeof(app_data.diversifier),
                 NVM_OFFSET_SM_DIV);
        
        /* Read device name and length from NVM */
        GapReadDataFromNVM(&offset);
        
    }
    else /* NVM Sanity check failed means either the device is being brought up 
          * for the first time or memory has got corrupted in which case discard 
          * the data and start fresh.
          */
    {
        nvm_sanity = NVM_SANITY_MAGIC;

        /* Write NVM Sanity word to the NVM */
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

        /* The device will not be bonded as it is coming up for the first time*/
        app_data.bonded = FALSE;

        /* Write bonded status to NVM */
        Nvm_Write((uint16*)&app_data.bonded, sizeof(app_data.bonded),
                                                        NVM_OFFSET_BONDED_FLAG);

        
        /* When the mouse is booting up for the first time after flashing the
         * image to it, it will not have bonded to any device. So, no LTK will
         * be associated with it. So, set the diversifier to 0
         */
        app_data.diversifier = 0;

        /* Write the same to NVM. */
        Nvm_Write(&app_data.diversifier, sizeof(app_data.diversifier),
                  NVM_OFFSET_SM_DIV);

        /* Write Gap data to NVM */
        GapInitWriteDataToNVM(&offset);

    }

    /* Read HID service data from NVM if the devices are bonded and  
     * update the offset with the number of word of NVM required by 
     * this service
     */
    HidReadDataFromNVM(app_data.bonded, &offset);

    /* Read Battery service data from NVM if the devices are bonded and  
     * update the offset with the number of word of NVM required by 
     * this service
     */
    BatteryReadDataFromNVM(app_data.bonded, &offset);

    /* Read Scan Parameter service data from NVM if the devices are bonded and  
     * update the offset with the number of word of NVM required by 
     * this service
     */
    ScanParamReadDataFromNVM(app_data.bonded, &offset);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      mouseDataInit
 *
 *  DESCRIPTION
 *      This function is used to initialise mouse application data.
 *      structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void mouseDataInit(void)
{
    /* Initialize the data structure variables used by the application to their
     * default values. Each service data has also to be initialized.
     */
    app_data.advert_timer_value = TIMER_INVALID;

    TimerDelete(app_data.app_tid);
    app_data.app_tid = TIMER_INVALID;
    
    TimerDelete(app_data.sensor_poll_tid);
    app_data.sensor_poll_tid = TIMER_INVALID;

    app_data.sensor_poll_period = INVALID_SENSOR_POLL_PERIOD;

    TimerDelete(app_data.conn_param_update_tid);
    app_data.conn_param_update_tid= TIMER_INVALID;
    app_data.cpu_timer_value = 0;

    /* Delete the bonding chance timer */
    TimerDelete(app_data.bonding_reattempt_tid);
    app_data.bonding_reattempt_tid = TIMER_INVALID;
                
    app_data.st_ucid = GATT_INVALID_UCID;

    app_data.encrypt_enabled = FALSE;

    app_data.pairing_button_pressed = FALSE;

    app_data.start_adverts = FALSE;

    app_data.waiting_for_fw_buffer = FALSE;

    app_data.connection_interval = 0;
    app_data.slave_latency = 0;
    app_data.supervision_timeout = 0;

    /* Initialise GAP Data Structure */
    GapDataInit();

    /* HID Service data initialization */
    HidDataInit();

    /* Battery Service data initialization */
    BatteryDataInit();

    /* Scan Parameter Service data initialization */
    ScanParamDataInit();

    /* Initialization of all reports of mouse */
    HwDataInit();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appEnterIdleState
 *
 *  DESCRIPTION
 *      This function is called when application is entering IDLE state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void appEnterIdleState(void)
{

    /* The mouse should start advertising when there is any
     * mouse movement. But the current consumption should also
     * be low when the mouse is not in use. So set the mouse
     * button PIOs to no_pulls but enable any mouse movement
     * detection.
     */
    app_data.hw_active = FALSE;
    StopHardware();
    ButtonDataInit();
    EnableMouseMotionAndButtonEvents();
}

#ifndef _NO_IDLE_TIMEOUT_

/*----------------------------------------------------------------------------*
 *  NAME
 *      mouseIdleTimerHandler
 *
 *  DESCRIPTION
 *      This function is used to handle IDLE timer.expiry in connected states.
 *      At the expiry of this timer, application shall disconnect with the 
 *      host and shall move to app_idle' state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void mouseIdleTimerHandler(timer_id tid)
{
    /* Trigger Disconnect and move to CON_DISCONNECTING state */

    if(tid == app_data.app_tid)
    {
        app_data.app_tid = TIMER_INVALID;

        if(app_data.state == app_connected)
        {
            appSetState(app_disconnecting);
        }
    } /* Else ignore the timer expiry, it may be due to a race condition */

}

#endif /* _NO_IDLE_TIMEOUT_ */

/*----------------------------------------------------------------------------*
 *  NAME
 *      resetIdleTimer
 *
 *  DESCRIPTION
 *      This function is used to restart the idle timer.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void resetIdleTimer(void)
{
    /* Reset Idle timer */
    TimerDelete(app_data.app_tid);

    app_data.app_tid = TIMER_INVALID;

#ifndef _NO_IDLE_TIMEOUT_

    app_data.app_tid = TimerCreate(CONNECTED_IDLE_TIMEOUT_VALUE, 
                                    TRUE, mouseIdleTimerHandler);

#endif /* _NO_IDLE_TIMEOUT_ */

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appEnterActiveState
 *
 *  DESCRIPTION
 *      This function is called when application is entering ACTIVE state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void appEnterActiveState(void)
{
    /* Delete the sensor polling timer if it is already running. */
    TimerDelete(app_data.sensor_poll_tid);
    app_data.sensor_poll_tid = TIMER_INVALID;
    
    /* Disable any further events due to mouse motion and wheel scroll as the
     * application keeps scanning for new data once any acknowledgement for
     * the transmitted data is received.
     */
    DisableMouseMotionAndWheelEvents();

    /* In ACTIVE state, the application has to start the sensor polling timer
     * when the connection event starts so that by the end of connection event,
     * new data, if present in the sensor is sent to the firmware which sends
     * it in the next connection event.
     */
    LsRadioEventNotification(app_data.st_ucid, radio_event_first_tx);

    /* Send the mouse report data */
    if(sendMouseReports())
    {
        /* Delete the idle timer. It will be re-created while coming back to
         * connected state
         */
        TimerDelete(app_data.app_tid);
        app_data.app_tid = TIMER_INVALID;

        /* Create the mouse sensor polling timer. */
        app_data.sensor_poll_tid = TimerCreate(app_data.sensor_poll_period,
                                            TRUE, sensorPollTimerExpiryHandler);
    }
    else
    {
        appSetState(app_connected);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appSetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
 
static void appSetState(app_state new_state)
{
    /* Check if the new state to be set is not the same as the present state
     * of the application. */
    uint16 old_state = app_data.state;
    
    if (old_state != new_state)
    {
        /* Handle exiting old state */
        switch (old_state)
        {
            case app_init:
                /* Common things to do while application exits app_init state.
                 * Application will start advertising upon exiting app_init
                 * state. So, update the whitelist.
                 */
                AppUpdateWhiteList();
            break;

            case app_idle:
                /* Common things to do whenever application exits
                 * app_idle state.
                 */
            break;

            case app_disconnecting:
                /* Common things to do whenever application exits
                 * app_disconnecting state.
                 */
            break;

            case app_active:
                /* Common things to do whenever application exits
                 * app_active state.
                 */
                /* Radio event notifications are received at every connection
                 * interval. These indications are no longer required as the
                 * application doesn't have any data to send. So, disable
                 * these events when exiting 'active' state.
                 */
                LsRadioEventNotification(app_data.st_ucid, radio_event_none);
                /* Wheel and wake pin events would have been disabled while
                 * entering active state. Re-enable these events
                 */
                ReEnableMouseEvents();
            break;

            default:
                /* Nothing to do. */    
            break;
        }

        /* Set new state */
        app_data.state = new_state;

        /* Handle entering new state */
        switch (new_state)
        {

            case app_direct_advert:
                /* Before starting to advertise, disable any further events
                 * because of a mouse button press or mouse motion
                 */
                DisableMouseEvents();
                /* Directed advertisement doesn't use any timer. Directed
                 * advertisements are done for 1.28 seconds always.
                 */
                app_data.advert_timer_value = TIMER_INVALID;
                GattStartAdverts(FALSE, gap_mode_connect_directed);
            break;

            case app_fast_advertising:
                /* Before starting to advertise, disable any further events
                 * because of a mouse button press or mouse motion
                 */
                DisableMouseEvents();
                GattTriggerFastAdverts();
            break;

            case app_slow_advertising:
                /* Enable detection of mouse motion and button presses so that
                 * if the user moves the mouse when it is in slow_advertising
                 * state, it can move to fast_advertising state
                 */
                EnableMouseMotionAndButtonEvents();
                
                GattStartAdverts(FALSE, gap_mode_connect_undirected);
            break;

            case app_idle:
                /* Common things to do whenever application enters
                 * app_idle state.
                 */
                appEnterIdleState();
            break;

            case app_connected:
                /* Common things to do whenever application enters
                 * app_connected state.
                 */
                resetIdleTimer();
            break;

            case app_active:
                /* Common things to do whenever application enters
                 * app_active state.
                 */
                appEnterActiveState();
            break;

            case app_disconnecting:
                GattDisconnectReq(app_data.st_ucid);
            break;

            default:
            break;
        }
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      appStartAdvert
 *
 *  DESCRIPTION
 *      This function is used to start directed advertisements if a valid
 *      reconnection address has been written by the remote device. Otherwise,
 *      it starts fast undirected advertisements.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void appStartAdvert(void)
{
    app_state advert_type;

    if(app_data.bonded && 
       !IsAddressResolvableRandom(&app_data.bonded_bd_addr) &&
       !IsAddressNonResolvableRandom(&app_data.bonded_bd_addr))
    {
        advert_type = app_direct_advert;

#ifdef __GAP_PRIVACY_SUPPORT__

        /* If re-connection address is not written, start fast undirected 
         * advertisements 
         */
        if(!GapIsReconnectionAddressValid())
        {
            advert_type = app_fast_advertising;
        }

#endif /* __GAP_PRIVACY_SUPPORT__ */
    }
    else /* Start with fast advertisements */
    {
        advert_type = app_fast_advertising;
    }

    appSetState(advert_type);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAddDBCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_ADD_DB_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void handleSignalGattAddDBCfm(void)
{
    switch(app_data.state)
    {
        case app_init:
        {
            appStartAdvert();
        }
        break;

        default:
            /*Control should never come here */
            ReportPanic(app_panic_invalid_state);
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattCancelConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CANCEL_CONNECT_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalGattCancelConnectCfm(void)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        /* GATT_CANCEL_CONNECT_CFM is received when undirected advertisements
         * are stopped.
         */
        case app_fast_advertising:
        case app_slow_advertising:
        {
            if(app_data.pairing_button_pressed)
            {
                /* Reset and clear the whitelist */
                LsResetWhiteList();

                mouseDataInit();
                
                if(app_data.state == app_fast_advertising)
                     GattTriggerFastAdverts();
                else
                    appSetState(app_fast_advertising);
            }
            else
            {
                if(app_data.state == app_fast_advertising)
                {
                    appSetState(app_slow_advertising);
                }
                /* Check whether we need to start directed or fast undirected
                 * advertisements due to user action on mouse
                 */
                else if(app_data.start_adverts)
                {
                    app_data.start_adverts = FALSE;

                    appStartAdvert();
                }
                else /* The application has finished slow advertising. So move
                      * the application to IDLE state.
                      */
                {                    
                    appSetState(app_idle);
                }
            }
        }
        break;
        
        default:
            /*Control should never come here */
            ReportPanic(app_panic_invalid_state);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CONNECT_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T* event_data)
{
    /*Handling signal as per current state */
    switch(app_data.state)
    {
        case app_fast_advertising:
        case app_slow_advertising:
        case app_direct_advert:
        {
            if(event_data->result == sys_status_success)
            {

                /* Delete the timer that was being used during advertisements */
                TimerDelete(app_data.app_tid);
                app_data.app_tid = TIMER_INVALID;
                
                app_data.advert_timer_value = TIMER_INVALID;
                
                /* Store received UCID */
                app_data.st_ucid = event_data->cid;

                if(app_data.bonded && 
                IsAddressResolvableRandom(&app_data.bonded_bd_addr) &&
                (SMPrivacyMatchAddress(&event_data->bd_addr,
                                        app_data.central_device_irk.irk,
                                        MAX_NUMBER_IRK_STORED, 
                                        MAX_WORDS_IRK) < 0))
                {
                    /* Application was bonded to a remote device with resolvable
                     * random address and application has failed to resolve the
                     * remote device address to which we just connected So
                     * disconnect and start advertising again
                     */

                    /* Disconnect if we are connected */
                    appSetState(app_disconnecting);
                }

                else
                {
                    app_data.con_bd_addr = event_data->bd_addr;

                    /* Request Security only if remote device address is not
                     * resolvable random. This is because APPLE iOS uses
                     * resolvalbe random address. Their spec says that the
                     * accessories should not send SLAVE_SECURITY_REQUEST.
                     */
                    if(!IsAddressResolvableRandom(&app_data.con_bd_addr))
                    {                    
                        SMRequestSecurityLevel(&app_data.con_bd_addr);
                    }

                    /* Create the connection parameter update timer for a period
                     * of TGAP(conn_pause_peripheral) if the present connection
                     * parameters are not the ones preferred by us.
                     */
                    app_data.conn_param_update_cnt = 0;
                    if(app_data.connection_interval < PREFERRED_MIN_CON_INTERVAL
                    || app_data.connection_interval > PREFERRED_MAX_CON_INTERVAL

#if PREFERRED_SLAVE_LATENCY

                       || app_data.slave_latency < PREFERRED_SLAVE_LATENCY

#endif /* PREFERRED_SLAVE_LATENCY */

                       )
                    {
                        TimerDelete(app_data.conn_param_update_tid);
                        app_data.conn_param_update_tid = TimerCreate(
                                                          TGAP_CPP_PERIOD, TRUE,
                                                       handleGapCppTimerExpiry);
                        app_data.cpu_timer_value = TGAP_CPP_PERIOD;
                    }

                    appSetState(app_connected);
                }                
            }
            else
            {

                if((event_data)->result == 
                    HCI_ERROR_DIRECTED_ADVERTISING_TIMEOUT)
                {
                    /* Case where bonding has been removed when directed 
                     * advertisements were on-going
                     */
                    if(app_data.pairing_button_pressed)
                    {
                        /* Reset and clear the whitelist */
                        LsResetWhiteList();
                    }

                    /* Trigger undirected fast advertisements as directed 
                     * advertisements have timed out.
                     */
                    appSetState(app_fast_advertising);
                }
                else
                {

                    /* Connection failure. */
                    ReportPanic(app_panic_connection_est);

                }
            }
        }
        break;
        
        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattCharValNotCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CHAR_VAL_NOT_CFM which is received
 *      as acknowledgement from the firmware that the data sent from application
 *      has been queued to be transmitted
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalGattCharValNotCfm(GATT_CHAR_VAL_IND_CFM_T *p_event_data)
{
    switch(app_data.state)
    {
        case app_active:
            /* If the firmware hasn't successfully queued the data sent by the
             * application(this happens in bad radio conditions/environment),
             * then stop sending any new data and wait for radio_event_tx_data
             * which indicates that the firmware has successfully sent some
             * data to the connected device.
             */
            if(p_event_data->result != sys_status_success)
            {
                app_data.waiting_for_fw_buffer = TRUE;

                /* Now, we shouldn't send new data upon sensor polling timer
                 * expiry. We should rather wait for radio_event_first_tx and
                 * re-try sending this data upon receiving this event.
                 */
                TimerDelete(app_data.sensor_poll_tid);
                app_data.sensor_poll_tid = TIMER_INVALID;
            }
            else
            {
                /* The packet was sent successfully - reset the firmware 
                 * queue flag.
                 */
                if(app_data.waiting_for_fw_buffer)
                {
                    app_data.waiting_for_fw_buffer = FALSE;
                }
            }
        break;
        
        default:
            /* Do nothing. */
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAccessInd
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_ACCESS_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *event_data)
{
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
            /* GATT_ACCESS_IND indicates that the central device is still disco-
             * -vering services. So, restart the connection parameter update ti-
             * -mer
             */
            if(app_data.cpu_timer_value == TGAP_CPC_PERIOD)
            {
                TimerDelete(app_data.conn_param_update_tid);
                app_data.conn_param_update_tid = TimerCreate(TGAP_CPC_PERIOD,
                                                 TRUE, requestConnParamUpdate);
            }
            GattHandleAccessInd(event_data);
        break;

        default:
            /* The remote side is trying to access some attribute in wrong
             * state. Ignore it.
             */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmEncryptionChange
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_ENCRYPTION_CHANGE
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmEncryptionChange(LM_EVENT_T *event_data)
{
    /*Handling signal as per current state */
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {            
            HCI_EV_DATA_ENCRYPTION_CHANGE_T *pEvDataEncryptChange = 
                                        &event_data->enc_change.data;

            if(pEvDataEncryptChange->status == HCI_SUCCESS)
            {
                app_data.encrypt_enabled = pEvDataEncryptChange->enc_enable;

                /* Delete the bonding chance timer */
                TimerDelete(app_data.bonding_reattempt_tid);
                app_data.bonding_reattempt_tid = TIMER_INVALID;
            }

            if(app_data.encrypt_enabled)
            {
                /* Initialize the hardware which will enable mouse motion and
                 * button press detection only if the mouse is bonded. If not,
                 * these events will be enabled after receiving
                 * SM_SIMPLE_PAIRING_COMPLETE_IND
                 */
                if(app_data.bonded)
                {
                    if(HidIsNotifyEnabledOnReportId(HID_INPUT_REPORT_ID))
                    {
                        /* Upon re-connection, if the system is in sleep, the
                         * mouse needs to send some data to wake the system up.
                         * But if all the data pertaining to the movement done
                         * by the user until encryption got enabled(which is
                         * accumulated in the sensor) is sent, it leads to the
                         * user seeing a sudden large jump of mouse cursor. So,
                         * send dummy data so that the system can wake up but
                         * the user doesn't see any large cursor jump
                         */
                        uint8 dummy_report[ATTR_LEN_HID_INPUT_REPORT];

                        MemSet(dummy_report, 0, ATTR_LEN_HID_INPUT_REPORT);
                        
                        HidSendInputReport(app_data.st_ucid,
                                           HID_INPUT_REPORT_ID, dummy_report);
                        /* No need to check whether the firmware queues are full
                         * since this is the first data that is being sent and
                         * next data will be sent only after there is new data
                         * again in the sensor which will take some more time
                         * after this(at present, it will take a minimum of 8ms,
                         * this time value can change depending upon the sensor
                         * configuration of mouse)
                         */
                    }
                
                    if(!app_data.hw_active)
                    {
                        app_data.hw_active = TRUE;
                        StartHardware();
                    }
                    ButtonDataInit();
                    ReEnableMouseEvents();
                }

                /* Update battery status at every connection instance. It
                 * may not be worth updating timer more often, but again it
                 * will primarily depend upon application requirements 
                 */
                BatteryUpdateLevel(app_data.st_ucid);

#ifndef _NO_IDLE_TIMEOUT_

                ScanParamRefreshNotify(app_data.st_ucid);

#endif /* _NO_IDLE_TIMEOUT_ */

            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmEvDisconnectComplete
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_DISCONNECT_COMPLETE.
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalLmEvDisconnectComplete(
                                HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        /* LM_EV_DISCONNECT_COMPLETE is received by the application when
         * 1. The remote side initiates a disconnection. The remote side can
         *    disconnect in any of the following states:
         *    a. app_connected.
         *    b. app_active.
         *    c. app_disconnecting.
         * 2. The mouse application itself initiates a disconnection. The
         *    mouse application will have moved to app_disconnecting state
         *    when it initiates a disconnection.
         * 3. There is a link loss.
         */
        case app_disconnecting:
        case app_connected:
        case app_active:
        {
            /* Initialize the mouse data structure. This will expect that the
             * remote side re-enables encryption on the re-connection though it
             * was a link loss.
             */
            mouseDataInit();

            /* If there was a link loss, the mouse needs to start advertising */
            if(p_event_data->reason == HCI_ERROR_CONN_TIMEOUT)
            {
                appStartAdvert();
            }
            /* If there wasn't a link loss, the disconnection can be
             * 1. Remote device initiated or
             * 2. Mouse initiated due to idle timeout
             * In these cases, if the mouse is not bonded to any host, it needs
             * to advertise. If it's already bonded, it can move to idle state
             */
            else if(!app_data.bonded)
            {
                appSetState(app_fast_advertising);
            }
            else
            {
                appSetState(app_idle);
            }
            
        }        
        break;
        
        default:
            /* Control should never reach here. */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmEvConnectionComplete
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_COMPLETE.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handleSignalLmEvConnectionComplete(
                                        LM_EV_CONNECTION_COMPLETE_T *event_data)
{

    /* Store the connection parameters. */
    app_data.connection_interval = event_data->data.conn_interval;
    app_data.slave_latency = event_data->data.conn_latency;
    app_data.supervision_timeout = event_data->data.supervision_timeout;

    /* Received from the LM layer in parallel to GATT_CONNECT_CFM. So
     * without checking the application state, update the sensor polling
     * period.
     */
    app_data.sensor_poll_period = event_data->data.conn_interval * 1250UL - 
                                  RESIDUAL_TIME;

    /* Update the rate at which the mouse sensor updates its registers. */
    SensorUpdateFrameRate(event_data->data.conn_interval);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmKeysInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_KEYS_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalSmKeysInd(SM_KEYS_IND_T *event_data)
{
    /*Handling signal as per current state */
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {
            /* If keys are present, save them */
            if((event_data->keys)->keys_present & (1 << SM_KEY_TYPE_DIV))
            {
                /* Store the diversifier which will be used for accepting/
                 * rejecting the encryption requests.
                 */
                app_data.diversifier = (event_data->keys)->div;
            
                /* Write the new diversifier to NVM */
                Nvm_Write(&app_data.diversifier,
                          sizeof(app_data.diversifier), NVM_OFFSET_SM_DIV);
            }

            /* Store IRK if the connected host is using random resolvable 
             * address. IRK is used afterwards to validate the identity of 
             * connected host 
             */

            if(IsAddressResolvableRandom(&app_data.con_bd_addr) &&
               ((event_data->keys)->keys_present & (1 << SM_KEY_TYPE_ID)))
            {
                MemCopy(app_data.central_device_irk.irk,
                                (event_data->keys)->irk,
                                MAX_WORDS_IRK);

                /* If bonded device address is resolvable random
                 * then store IRK in NVM 
                 */
                Nvm_Write(app_data.central_device_irk.irk, 
                                MAX_WORDS_IRK, NVM_OFFSET_SM_IRK);
            }
        }
        break;
        
        default:
            ReportPanic(app_panic_invalid_state);
    }
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmPairingAuthInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_PAIRING_AUTH_IND. This message will
 *      only be received when the peer device is initiating 'Just Works' 
 *      pairing.
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalSmPairingAuthInd(SM_PAIRING_AUTH_IND_T *p_event_data)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        case app_connected:
        {
            /* Authorise the pairing request if the Keyboard is NOT bonded */
            if(!app_data.bonded)
            {
                SMPairingAuthRsp(p_event_data->data, TRUE);
            }
            else /* Otherwise Reject the pairing request */
            {
                SMPairingAuthRsp(p_event_data->data, FALSE);
            }
        }
        break;

        default:
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmDivApproveInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_DIV_APPROVE_IND.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handleSignalSmDivApproveInd(SM_DIV_APPROVE_IND_T *p_event_data)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        
        /* Request for approval from application comes only when pairing is not
         * in progress. So, this event can't come in app_active state
         */
        case app_connected:
        {
            sm_div_verdict approve_div = SM_DIV_REVOKED;
            
            /* Check whether the application is still bonded(bonded flag gets
             * reset upon 'connect' button press by the user). Then check
             * whether the diversifier is the same as the one stored by the
             * application
             */
            if(app_data.bonded)
            {
                if(app_data.diversifier == p_event_data->div)
                {
                    approve_div = SM_DIV_APPROVED;
                }
            }

            SMDivApproval(p_event_data->cid, approve_div);
        }
        break;

        default:
            ReportPanic(app_panic_invalid_state);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmSimplePairingCompleteInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_SIMPLE_PAIRING_COMPLETE_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *event_data)
{
    /*Handling signal as per current state */
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {            
            if(event_data->status == sys_status_success)
            {
                app_data.bonded = TRUE;
                app_data.bonded_bd_addr = event_data->bd_addr;

                /* Store bonded host typed bd address to NVM */

                /* Write one word bonded flag */
                Nvm_Write((uint16*)&app_data.bonded, sizeof(app_data.bonded),
                                                     NVM_OFFSET_BONDED_FLAG);

                /* Write typed bd address of bonded host */
                Nvm_Write((uint16*)&app_data.bonded_bd_addr,
                          sizeof(TYPED_BD_ADDR_T), NVM_OFFSET_BONDED_ADDR);

                /* White list is configured with the Bonded host address */
                AppUpdateWhiteList();

                if(!app_data.hw_active)
                {
                    app_data.hw_active = TRUE;
                    StartHardware();
                }
                ButtonDataInit();
                ReEnableMouseEvents();
            }
            /* If the pairing failed and it was not just a disconnection
             * initiated from the remote device, remove bonding information
             */
            else
            {
                /* Pairing has failed.
                 * 1. If pairing has failed due to repeated attempts, the 
                 *    application should immediately disconnect the link.
                 * 2. The application was bonded and pairing has failed.
                 *    Since the application was using whitelist, so the remote 
                 *    device has same address as our bonded device address.
                 *    The remote connected device may be a genuine one but 
                 *    instead of using old keys, wanted to use new keys. We 
                 *    don't allow bonding again if we are already bonded but we
                 *    will give some time to the connected device to encrypt the
                 *    link using the old keys. if the remote device encrypts the
                 *    link in that time, it's good. Otherwise we will disconnect
                 *    the link.
                 */
                 if(event_data->status == sm_status_repeated_attempts)
                 {
                    appSetState(app_disconnecting);
                 }
                 else if(app_data.bonded)
                 {
                    app_data.encrypt_enabled = FALSE;
                    app_data.bonding_reattempt_tid = 
                                          TimerCreate(
                                               BONDING_CHANCE_TIMER,
                                               TRUE, 
                                               handleBondingChanceTimerExpiry);
                 }
            }
        }
        break;
        
        default:
            /* SM_SIMPLE_PAIRING_COMPLETE_IND reaches the application after
             * LM_EV_DISCONNECT_COMPLETE when the remote end terminates
             * the connection without enabling encryption or completing pairing.
             * The application will be either in advertising or idle state in
             * this scenario. So, don't panic
             */
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnParamUpdateCfm
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_CFM.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalLsConnParamUpdateCfm(
                            LS_CONNECTION_PARAM_UPDATE_CFM_T *event_data)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {
            if ((event_data->status !=
                    ls_err_none) && (app_data.conn_param_update_cnt < 
                    MAX_NUM_CONN_PARAM_UPDATE_REQS))
            {
                app_data.conn_param_update_tid = 
                    TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                TRUE, requestConnParamUpdate);
                app_data.cpu_timer_value = GAP_CONN_PARAM_TIMEOUT;
            }
        }
        break;
        
        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnectionParamUpdateInd
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_IND.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void handleSignalLsConnectionParamUpdateInd(LM_EVENT_T* event_data)
{
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {
            /* connection parameters update */
            LS_CONNECTION_PARAM_UPDATE_IND_T* event = 
                                  (LS_CONNECTION_PARAM_UPDATE_IND_T*)event_data;

            /* Delete timer if running */
            TimerDelete(app_data.conn_param_update_tid);
            app_data.conn_param_update_tid = TIMER_INVALID;
            app_data.cpu_timer_value = 0;

            /* Connection parameters have been updated. Check if new parameters 
             * comply with application preffered parameters. If not, application
             * shall trigger Connection parameter update procedure 
             */
            if(event->conn_interval < PREFERRED_MIN_CON_INTERVAL ||
                event->conn_interval > PREFERRED_MAX_CON_INTERVAL

#if PREFERRED_SLAVE_LATENCY

                || event->conn_latency < PREFERRED_SLAVE_LATENCY

#endif /* PREFERRED_SLAVE_LATENCY */

                )
            {
                /* Set the connection parameter update attempts counter to 
                 * zero
                 */
                app_data.conn_param_update_cnt = 0;

                /* Start timer to trigger Connection Paramter Update 
                 * procedure 
                 */
                app_data.conn_param_update_tid = 
                            TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                        TRUE, requestConnParamUpdate);
                app_data.cpu_timer_value = GAP_CONN_PARAM_TIMEOUT;
            }

        }
        break;

        default:
            /* Connection parameter update indication received in unexpected
             * application state.
             */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmConnectionUpdate
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_UPDATE.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmConnectionUpdate(
                                          LM_EV_CONNECTION_UPDATE_T* event_data)
{
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {
            /* Store the connection parameters. */
            app_data.connection_interval = event_data->data.conn_interval;
            app_data.slave_latency = event_data->data.conn_latency;
            app_data.supervision_timeout = event_data->data.supervision_timeout;

            /* Update the sensor polling period. */
            app_data.sensor_poll_period = 
                        event_data->data.conn_interval * 1250UL - RESIDUAL_TIME;

            /* Update the rate at which the mouse sensor updates its registers.
             */
            SensorUpdateFrameRate(event_data->data.conn_interval);
        }
        break;

        default:
            /* Connection parameter update indication received in unexpected
             * application state.
             */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsRadioEventInd
 *
 *  DESCRIPTION
 *      This function handles the signal LS_RADIO_EVENT_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

static void handleSignalLsRadioEventInd(void)
{
    /* Handling signal as per current state */
    switch(app_data.state)
    {
        /* Reception of radio_event_tx_data event indicates successful
         * transmission of data to the remote device. This should have freed up
         * some buffer space in f/w for transmitting another mouse input report.
         */
        case app_active:
            if(app_data.waiting_for_fw_buffer)
            {
                /* If a data packet has been sent by firmware, application can
                 * re-try sending the last data which had failed.
                 */
                HidSendInputReport(app_data.st_ucid, HID_INPUT_REPORT_ID,
                                           GetMouseReport(HID_INPUT_REPORT_ID));                
            }
            /* The sensor polling timer should be started at the start of
             * connection interval for us to be sure that it expires once in
             * one connection interval. So, align it with radio_event_first_tx.
             */
            TimerDelete(app_data.sensor_poll_tid);
            app_data.sensor_poll_tid = TimerCreate(app_data.sensor_poll_period,
                                            TRUE, sensorPollTimerExpiryHandler);
        break;

        /* If the radio event indicating a data packet transmission is received
         * in any other states, ignore the event
         */
        default:
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleMouseMotionEvent
 *
 *  DESCRIPTION
 *      This function is called upon detecting a mouse motion by the user.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleMouseMotionEvent(void)
{
    switch(app_data.state)
    {
        case app_connected:
            appSetState(app_active);
        break;

        case app_idle:
            appStartAdvert();
        break;

        case app_slow_advertising:
        {
            /* Disable any further events due to mouse motion */
            DisableMouseEvents();

            /* Set the flag that indicates that fast/directed advertisements 
             * should be started upon receiving GATT_CANCEL_CONNECT_CFM
             */
            app_data.start_adverts = TRUE;

            /* Delete the advertising timer */
            TimerDelete(app_data.app_tid);
            app_data.app_tid = TIMER_INVALID;
            app_data.advert_timer_value = TIMER_INVALID;

            GattStopAdverts();
        }
        break;

        /* If mouse motion is detected in any other state, ignore the data
         * but re-enable the mouse motion detection.
         */
        default:
            EnableMouseMotionDetection();
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleBatteryLowEvent
 *
 *  DESCRIPTION
 *      This function is called upon detecting a battery low system event.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleBatteryLowEvent(battery_low_data *data)
{
    switch(app_data.state)
    {
        case app_connected:
        case app_active:
        {
            /* Notify the remote device(Host) that mouse's battery level is
             * critical. Additionally, an LED can be configured to report 
             * critically low battery level and glown upon receiving this event.
             */
            BatteryUpdateLevel(app_data.st_ucid);

            /* Start glowing the low power LED. */
            if(data->is_below_threshold)
            {
                GlowLowPowerLED(TRUE);
            }
            else
            {
                GlowLowPowerLED(FALSE);
            }
        }
        break;

        default:
            /* If the event is received in any other state such as advertising,
             * ignore it.
             */
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleGapCppTimerExpiry
 *
 *  DESCRIPTION
 *      This function handles the expiry of TGAP(conn_pause_peripheral) timer.
 *      It starts the TGAP(conn_pause_central) timer, during which, if no activ-
 *      -ity is detected from the central device, a connection parameter update
 *      request is sent.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleGapCppTimerExpiry(timer_id tid)
{
    if(app_data.conn_param_update_tid == tid)
    {
        app_data.conn_param_update_tid = TimerCreate(TGAP_CPC_PERIOD, TRUE,
                                                     requestConnParamUpdate);
        app_data.cpu_timer_value = TGAP_CPC_PERIOD;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      requestConnParamUpdate
 *
 *  DESCRIPTION
 *      This function is used to send L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST
 *      to the remote device when an earlier sent request had failed.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void requestConnParamUpdate(timer_id tid)
{
    ble_con_params app_pref_conn_params;

    if(app_data.conn_param_update_tid == tid)
    {
        app_data.conn_param_update_tid= TIMER_INVALID;
        app_data.cpu_timer_value = 0;

        /* Decide which parameter values are to be used. */
        if(app_data.conn_param_update_cnt < CPU_SELF_PARAMS_MAX_ATTEMPTS)
        {
            app_pref_conn_params.con_max_interval = 
                                        PREFERRED_MAX_CON_INTERVAL;
            app_pref_conn_params.con_min_interval = 
                                        PREFERRED_MIN_CON_INTERVAL;
            app_pref_conn_params.con_slave_latency = 
                                        PREFERRED_SLAVE_LATENCY;
            app_pref_conn_params.con_super_timeout = 
                                        PREFERRED_SUPERVISION_TIMEOUT;
        }
        else
        {
            app_pref_conn_params.con_max_interval = 
                                        APPLE_MAX_CON_INTERVAL;
            app_pref_conn_params.con_min_interval = 
                                        APPLE_MIN_CON_INTERVAL;
            app_pref_conn_params.con_slave_latency = 
                                        APPLE_SLAVE_LATENCY;
            app_pref_conn_params.con_super_timeout = 
                                        APPLE_SUPERVISION_TIMEOUT;
        }

        /* Send a connection parameter update request only if the remote device
         * has not entered 'suspend' state.
         */
        if(!HidIsStateSuspended())
        {
            ++ app_data.conn_param_update_cnt;
            LsConnectionParamUpdateReq(&(app_data.con_bd_addr), 
                                                     &app_pref_conn_params);
        }
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      sendMouseReports
 *
 *  DESCRIPTION
 *      Sends the updated HID reports coherent with LS_RADIO_EVENT_IND.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
 
static bool sendMouseReports(void)
{
    hid_protocol_mode protocol_mode = GetReportMode();
    uint8 temp_input_report[ATTR_LEN_LARGEST_REPORT];
    bool data_sent = FALSE;
    
    /* Send a new data if it's available. */
    if(GetMouseData((uint8)protocol_mode, temp_input_report))
    {
        if(HidIsNotifyEnabledOnReportId(HID_INPUT_REPORT_ID))
        {
            HidSendInputReport(app_data.st_ucid, HID_INPUT_REPORT_ID,
                                   temp_input_report);

            data_sent = TRUE;
        }
    }
    return data_sent;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      sensorPollTimerExpiryHandler
 *
 *  DESCRIPTION
 *      This function is used to handle the expiry of sensor poll timer. Upon
 *      the expiry of this timer, data is sent to the connected HID host.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void sensorPollTimerExpiryHandler(timer_id tid)
{
    if(tid == app_data.sensor_poll_tid)
    {
        app_data.sensor_poll_tid = TIMER_INVALID;
        switch(app_data.state)
        {
            case app_active:
                if(sendMouseReports())
                {
                    /* Create the mouse sensor polling timer. */
                    app_data.sensor_poll_tid = TimerCreate(
                                            app_data.sensor_poll_period,
                                            TRUE, sensorPollTimerExpiryHandler);
                }
                else
                {
                    appSetState(app_connected);
                }
            break;

            default:
                /* Do nothing. */
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleBondingChanceTimerExpiry
 *
 *  DESCRIPTION
 *      This function is handle the expiry of bonding chance timer.
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 
*----------------------------------------------------------------------------*/
static void handleBondingChanceTimerExpiry(timer_id tid)
{
    if(app_data.bonding_reattempt_tid== tid)
    {
        app_data.bonding_reattempt_tid= TIMER_INVALID;
        /* The bonding chance timer has expired. This means the remote has not
         * encrypted the link using old keys. Disconnect the link.
         */
        appSetState(app_disconnecting);
    }/* Else it may be due to some race condition. Ignore it. */
}


/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      ReportPanic
 *
 *  DESCRIPTION
 *      This function raises the panic.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
extern void ReportPanic(app_panic_code panic_code)
{
    /* If we want any debug prints, we can put them here */
    Panic(panic_code);
}



#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteApplicationAndServiceDataToNVM
 *
 *  DESCRIPTION
 *      This function writes the application data to NVM. This function should 
 *      be called on getting nvm_status_needs_erase
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

extern void WriteApplicationAndServiceDataToNVM(void)
{
    uint16 nvm_sanity = 0xffff;
    nvm_sanity = NVM_SANITY_MAGIC;

    /* Write NVM sanity word to the NVM */
    Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

    /* Write Bonded flag to NVM. */
    Nvm_Write((uint16*)&app_data.bonded, 
               sizeof(app_data.bonded),
               NVM_OFFSET_BONDED_FLAG);


    /* Write Bonded address to NVM. */
    Nvm_Write((uint16*)&app_data.bonded_bd_addr,
              sizeof(TYPED_BD_ADDR_T),
              NVM_OFFSET_BONDED_ADDR);

    /* Write the diversifier to NVM */
    Nvm_Write(&app_data.diversifier,
                sizeof(app_data.diversifier),
                NVM_OFFSET_SM_DIV);

    /* Store the IRK to NVM */
    Nvm_Write(app_data.central_device_irk.irk,
                MAX_WORDS_IRK,
                NVM_OFFSET_SM_IRK);

    /* Write GAP service data into NVM */
    WriteGapServiceDataInNVM();

    /* Write HID service data into NVM */
    WriteHIDServiceDataInNvm();

    /* Write Battery service data into NVM */
    WriteBatteryServiceDataInNvm();

    /* Write Scan Parameter Service data into NVM. */
    WriteScanParamServiceDataInNvm();
}
#endif /* NVM_TYPE_FLASH */



/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppUpdateWhiteList
 *
 *  DESCRIPTION
 *      This function updates the whitelist with bonded device address if
 *      it's not private, and also reconnection address when it has been written
 *      by the remote device.
 *
 *----------------------------------------------------------------------------*/

extern void AppUpdateWhiteList(void)
{
    LsResetWhiteList();
    
    if(app_data.bonded)
    {
        if(!IsAddressResolvableRandom(&app_data.bonded_bd_addr) &&
           !IsAddressNonResolvableRandom(&app_data.bonded_bd_addr))
        {
            /* If the device is bonded and bonded device address is not private
             * (resolvable random or non-resolvable random), configure
             * White list with the Bonded host address 
             */
             
            if(LsAddWhiteListDevice(&app_data.bonded_bd_addr) != ls_err_none)
            {
                ReportPanic(app_panic_add_whitelist);
            }
        }

#ifdef __GAP_PRIVACY_SUPPORT__

        if(GapIsReconnectionAddressValid())
        {
            TYPED_BD_ADDR_T temp_addr;

            temp_addr.type = ls_addr_type_random;
            MemCopy(&temp_addr.addr, GapGetReconnectionAddress(),
                                                            sizeof(BD_ADDR_T));
            if(LsAddWhiteListDevice(&temp_addr) != ls_err_none)
            {
                ReportPanic(app_panic_add_whitelist);
            }
        }

#endif /* __GAP_PRIVACY_SUPPORT__ */
    }

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppIsDeviceBonded
 *
 *  DESCRIPTION
 *      This function returns the status wheather the connected device is 
 *      bonded or not.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern bool AppIsDeviceBonded(void)
{
    return app_data.bonded;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HandleMousePioStatusChange
 *
 *  DESCRIPTION
 *      This function is called upon detecting a mouse button
 *      press(left, right or middle).
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

extern void HandleMousePioStatusChange(void)
{
    switch(app_data.state)
    {
        case app_connected:
        {
            appSetState(app_active);
        }
        break;
        
        case app_idle:
            appStartAdvert();
        break;

        case app_slow_advertising:
        {
            /* Disable any further events due to mouse motion */
            DisableMouseEvents();

            /* Set the flag to indicate that directed / fast advertisements 
             * should be started upon receiving GATT_CANCEL_CONNECT_CFM
             */
            app_data.start_adverts = TRUE;

            /* Delete the advertising timer */
            TimerDelete(app_data.app_tid);
            app_data.app_tid = TIMER_INVALID;
            app_data.advert_timer_value = TIMER_INVALID;

            GattStopAdverts();
        }
        break;

        default:
            /* The mouse button press/scroll event, if received in 'app_active'
             * state, the data will be sent in the next report. In other states,
             * the events can be ignored
             */
         break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      HandlePairingButtonPress
 *
 *  DESCRIPTION
 *      This function is called when the pairing removal button is pressed down
 *      for a period specified by PAIRING_REMOVAL_TIMEOUT
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
extern void HandlePairingButtonPress(timer_id tid)
{

    if(tid == pairing_removal_tid)
    {
        /* The firmware will have deleted the pairing removal timer. Set the
         * value of the timer maintained by application to TIMER_INVALID
         */
        pairing_removal_tid = TIMER_INVALID;

        /* Handle pairing button press only if the last pairing button press
         * handling is complete(app_data.pairing_button_pressed will be set
         * to false in such a case)
         */
        if(!app_data.pairing_button_pressed)
        {
            /* Pairing button pressed for PAIRING_REMOVAL_TIMEOUT period -
             * Remove bonding information
             */

            app_data.bonded = FALSE;

            /* Update the bonding status in NVM */
            Nvm_Write((uint16*)&app_data.bonded, sizeof(app_data.bonded),
                      NVM_OFFSET_BONDED_FLAG);

            switch(app_data.state)
            {        
                /* Disconnect if we are connected */
                case app_connected:
                case app_active:
                {
                    LsResetWhiteList();
                    appSetState(app_disconnecting);
                }
                break;

                case app_fast_advertising:
                case app_slow_advertising:
                case app_direct_advert:
                {
                    app_data.pairing_button_pressed = TRUE;

                    if(app_data.state != app_direct_advert)
                    {
                        /* Delete the advertising timer as in race conditions,
                         * it may expire before GATT_CANCEL_CONNECT_CFM reaches
                         * the application. If this happens, we end up calling
                         * GattStopAdverts() again
                         */

                        TimerDelete(app_data.app_tid);
                        app_data.app_tid = TIMER_INVALID;
                        app_data.advert_timer_value = TIMER_INVALID;

                        GattStopAdverts();
                    }
                }
                break;

                case app_idle:
                {
                    LsResetWhiteList();
                    appSetState(app_fast_advertising);
                }
                break;

                default:
                    /* If the application is in app_disconnecting/ app_idle
                     * states, it'll start advertising upon disconnection is
                     * complete as it'll find that the application is no
                     * more bonded to any device
                     */
                break;
                    
            }
        }
    } /* Else, ignore the function call as it may be due to a race condition */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This function is called just after a power-on reset (including after
 *      a firmware panic).
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the reset() function.
 *
 *      The last sleep state is provided to the application in the parameter.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppPowerOnReset(void)
{
    /* Configure the application constants */
}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This function is called after a power-on reset (including after a
 *      firmware panic) or after an HCI Reset has been requested.
 *      Contains the initialization sequence.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state LastSleepState)
{
    uint16 gatt_database_length;
    uint16 *p_gatt_db_ptr = NULL;

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);

    /* Initialise GATT entity */
    GattInit();

    /* Install GATT Server support for the optional Write procedures */
    GattInstallServerWrite();

#ifdef NVM_TYPE_EEPROM
    /* Configure the NVM manager to use I2C EEPROM for NVM store */
    NvmConfigureI2cEeprom();
#elif NVM_TYPE_FLASH
    /* Configure the NVM Manager to use SPI flash for NVM store. */
    NvmConfigureSpiFlash();
#endif /* NVM_TYPE_EEPROM */

    Nvm_Disable();
    
    GapDataInit();
    
    /* HID Service initialization on Chip reset */
    HidInitChipReset();

    /* Battery Service initialization on Chip reset */
    BatteryInitChipReset();

    /* Scan Parameter Service initialization on Chip reset */
    ScanParamInitChipReset();

    /* Read persistent storage */
    readPersistentStore();

    /* Tell Security Manager module about the value it needs to initialize it's
     * diversifier to.
     */
    SMInit(app_data.diversifier);

    /* Initialise mouse application data structure */
    mouseDataInit();

    /* The mouse button press and motion detection is not yet enabled. If the
     * application doesn't connect to any device, it's enabled after finishing
     * advertising. If the mouse gets connected, it's enabled after 
     * starting encryption.
     */
    app_data.hw_active = FALSE;

    /* Initialise Hardware components to their initial configurations. */
    InitHardware();

    /* Initialise mouse state */
    app_data.state = app_init;
    
    /* Tell GATT about our database. We will get a GATT_ADD_DB_CFM event when
     * this has completed.
     */
    p_gatt_db_ptr = GattGetDatabase(&gatt_database_length);
    GattAddDatabaseReq(gatt_database_length, p_gatt_db_ptr);

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppProcessSystemEvent(sys_event_id id, void *data)
{

    switch(id)
    {    
        case(sys_event_pio_changed):
        {
            HandlePIOChangedEvent(((pio_changed_data*)data)->pio_cause);
        }
        break;

        case sys_event_wakeup:
            handleMouseMotionEvent();
        break;

        case sys_event_battery_low:
            handleBatteryLowEvent((battery_low_data*)data);
        break;
        
        case sys_event_pio_ctrlr:
            /* Events from the PIO controller are due to the state changes in
             * wheel PIOs. When the wheel is scrolled, enter active state and
             * send the new data
             */
            HandleMousePioStatusChange();
        break;
        
        default:
            /* Any other system events, if they are of interest to the
             * particular application can be handled here.
             */
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/

bool AppProcessLmEvent(lm_event_code event_code, LM_EVENT_T *event_data)
{
    switch(event_code)
    {
        /* Handle events received from Firmware */

        /* Below messages are received in app_init state */
        case GATT_ADD_DB_CFM:
            /* Attribute database registration confirmation */
            handleSignalGattAddDBCfm();
        break;

        case GATT_CANCEL_CONNECT_CFM:
            /* Confirmation for the completion of GattCancelConnectReq()
             * procedure 
             */
            handleSignalGattCancelConnectCfm();
        break;
        
        /* Below messages are received in advertising state. */
        case GATT_CONNECT_CFM:
            /* Confirmation for the completion of GattConnectReq() 
             * procedure
             */
            handleSignalGattConnectCfm((GATT_CONNECT_CFM_T*)event_data);
        break;

        case LM_EV_CONNECTION_COMPLETE:
            handleSignalLmEvConnectionComplete(
                                     (LM_EV_CONNECTION_COMPLETE_T*)event_data);
        break;

        case SM_SIMPLE_PAIRING_COMPLETE_IND:
            /* Indication for completion of Pairing procedure */
            handleSignalSmSimplePairingCompleteInd(
                (SM_SIMPLE_PAIRING_COMPLETE_IND_T*)event_data);
        break;

        case LM_EV_ENCRYPTION_CHANGE:
            /* Indication for encryption change event */
            handleSignalLmEncryptionChange(event_data);
        break;

        case SM_KEYS_IND:
            /* Indication for the keys and associated security information
             * on a connection that has completed Short Term Key Generation 
             * or Transport Specific Key Distribution
             */
            handleSignalSmKeysInd((SM_KEYS_IND_T*)event_data);
        break;

        case SM_PAIRING_AUTH_IND:
            /* Authorize or Reject the pairing request */
            handleSignalSmPairingAuthInd((SM_PAIRING_AUTH_IND_T*)event_data);
        break;

        /* Received in response to the L2CAP_CONNECTION_PARAMETER_UPDATE request
         * sent from the slave after encryption is enabled. If the request has
         * failed, the device should again send the same request only after
         * Tgap(conn_param_timeout). Refer Bluetooth 4.0 spec Vol 3 Part C,
         * Section 9.3.9 and HID over GATT profile spec section 5.1.2.         
         */
        case LS_CONNECTION_PARAM_UPDATE_CFM:
            handleSignalLsConnParamUpdateCfm((LS_CONNECTION_PARAM_UPDATE_CFM_T*)
                event_data);
        break;

        case LS_CONNECTION_PARAM_UPDATE_IND:
            /* Indicates completion of remotely triggered Connection 
             * parameter update procedure
             */
            handleSignalLsConnectionParamUpdateInd(event_data);
        break;

        case LS_RADIO_EVENT_IND:
            handleSignalLsRadioEventInd();
        break;

        case GATT_ACCESS_IND:
            /* Indicates that an attribute controlled directly by the
             * application (ATT_ATTR_IRQ attribute flag is set) is being 
             * read from or written to.
             */
            handleSignalGattAccessInd((GATT_ACCESS_IND_T*)event_data);
        break;

        case LM_EV_DISCONNECT_COMPLETE:
            handleSignalLmEvDisconnectComplete(
            &((LM_EV_DISCONNECT_COMPLETE_T *)event_data)->data);
        break;

        case SM_DIV_APPROVE_IND:
            handleSignalSmDivApproveInd((SM_DIV_APPROVE_IND_T *)event_data);
        break;

        case LM_EV_NUMBER_COMPLETED_PACKETS:
        break;
      
        case LM_EV_CONNECTION_UPDATE:
            handleSignalLmConnectionUpdate(
                                        (LM_EV_CONNECTION_UPDATE_T*)event_data);
        break;

        case GATT_CHAR_VAL_NOT_CFM:
            handleSignalGattCharValNotCfm((GATT_CHAR_VAL_IND_CFM_T*)event_data);
        break;
        
        default:
        break;

    }

    return TRUE;
}
