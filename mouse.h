/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      mouse.h  -  user application for a BTLE mouse
 *
 *  DESCRIPTION
 *      Header file for a sample mouse application.
 *
 ******************************************************************************/

#ifndef _MOUSE_H_
#define _MOUSE_H_

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bluetooth.h>
#include <timer.h>

/*=============================================================================*
 *  Local Header File
 *============================================================================*/

#include "app_gatt.h"
#include "app_gatt_db.h"
#include "user_config.h"

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Maximum number of words in central device IRK */
#define MAX_WORDS_IRK                   (8)

/*Number of IRKs that application can store */
#define MAX_NUMBER_IRK_STORED           (1)

/* The status of buttons in input report of mouse when no buttons are pressed. */
#define ALL_BUTTONS_RELEASED            (0)

/*=============================================================================*
 *  Public Data Types
 *============================================================================*/

typedef enum
{

    app_init = 0,           /* Initial State */
    app_direct_advert,       /* This state is entered while reconnecting to
                             * a bonded host that does not support random 
                             * resolvable address. When privacy feature is 
                             * enabled, this state is entered when the remote 
                             * host has written to the re-connection address 
                             * characteristic during the last connection
                             */
    app_fast_advertising,   /* Fast Undirected advertisements configured */
    app_slow_advertising,   /* Slow Undirected advertisements configured */
    app_connected,          /* Mouse has established connection to the host */
    app_active,             /* Mouse is in motion and data is being transferred
                             * to host
                             */
    app_disconnecting,      /* Enters when disconnect is initiated by the 
                             * HID device
                             */
    app_idle               /* Enters when no user activity on the connected
                             * mouse for vendor specific time defined by
                             * FAST_CONNECTION_ADVERT_TIMEOUT_VALUE and 
                             * SLOW_CONNECTION_ADVERT_TIMEOUT_VALUE macro.
                             */
} app_state;

/* Structure defined for Central device IRK */
typedef struct
{
    uint16 irk[MAX_WORDS_IRK];

} CENTRAL_DEVICE_IRK_T;

typedef struct
{
    app_state state;

    /* Store timer id in 'UNDIRECTED ADVERTS', 'DIRECTED ADVERTS' and 
     * 'CONNECTED' states.
     */

    /* Value for which advertisement timer needs to be started. 
     *
     * For bonded devices, the timer is initially started for 30 seconds to 
     * enable fast connection by bonded device to the sensor.
     * This is then followed by reduced power advertisements for 1 minute.
     */
    uint32 advert_timer_value;

    /* Store timer id in 'FAST_ADVERTISING', 'SLOW_ADVERTISING' and 
     * 'CONNECTED' states.
     */
    timer_id app_tid;

    /* Mouse should send data at the rate of 1 report per connection interval.
     * This timer indicates the duration for which the mouse application should
     * be idle in a connection interval before polling the sensor for new data.
     */
    timer_id sensor_poll_tid;

    /* The timer value of 'sensor_poll_tid' */
    uint32 sensor_poll_period;

    /* Timer to hold the time elapsed after the last
     * L2CAP_CONNECTION_PARAMETER_UPDATE Request failed.
     */
    timer_id conn_param_update_tid;

    /* Connection Parameter Update timer value. Upon a connection, it's started
     * for a period of TGAP_CPP_PERIOD, upon the expiry of which it's restarted
     * for TGAP_CPC_PERIOD. When this timer is running, if a GATT_ACCESS_IND is
     * received, it means, the central device is still doing the service discov-
     * -ery procedure. So, the connection parameter update timer is deleted and
     * recreated. Upon the expiry of this timer, a connection parameter update
     * request is sent to the central device.
     */
    uint32 cpu_timer_value;

    /* A counter to keep track of the number of times the application has tried 
     * to send L2CAP connection parameter update request. When this reaches 
     * MAX_NUM_CONN_PARAM_UPDTAE_REQS, the application stops re-attempting to
     * update the connection parameters */
    uint8 conn_param_update_cnt;

    /* TYPED_BD_ADDR_T of the host to which mouse is connected */
    TYPED_BD_ADDR_T con_bd_addr;

    /* Track the UCID as Clients connect and disconnect */
    uint16 st_ucid;

    /* Boolean flag to indicated whether the device is bonded */
    bool bonded;

    /* TYPED_BD_ADDR_T of the host to which mouse is bonded. */
    TYPED_BD_ADDR_T bonded_bd_addr;

    /* Diversifier associated with the LTK of the bonded device */
    uint16 diversifier;

    /* Central Private Address Resolution IRK  Will only be used when
     * central device used resolvable random address.
     */
    CENTRAL_DEVICE_IRK_T central_device_irk;

    /* Boolean flag to indicate whether encryption is enabled with the bonded
     * host
     */
    bool encrypt_enabled;

    /* Boolean flag set to indicate pairing button press */
    bool pairing_button_pressed;

    /* If the mouse is moved in slow advertising state, the application shall 
     * move to directed or fast advertisements state. This flag is used to 
     * know whether the application needs to move to directed or fast 
     * advertisements state while handling GATT_CANCEL_CONNECT_CFM message.
     */
    bool start_adverts;

    /* Boolean flag set to indicate the mouse motion sensor and button press
     * sensing hardware is initialized.
     */
    bool hw_active;

    /* Boolean which indicates that the application is waiting for the firmware
     * to free its buffers. This flag is set when the application tries to add
     * some data(which has to be sent to the remote device) to the the firmware
     * and receives a gatt_status_busy response in GATT_CHAR_VAL_NOT_CFM.
     */
    bool waiting_for_fw_buffer;

    /* This timer will be used if the application is already bonded to the 
     * remote host address but the remote device wanted to rebond which we had 
     * declined. In this scenario, we give ample time to the remote device to 
     * encrypt the link using old keys. If remote device doesn't encrypt the 
     * link, we will disconnect the link on this timer expiry.
     */
    timer_id bonding_reattempt_tid;

    /* Variable to store the current connection interval being used. */
    uint16 connection_interval;

    /* Variable to store the current slave latency. */
    uint16 slave_latency;

    /* Variable to store the current supervision timeout value. */
    uint16 supervision_timeout;


} APP_DATA_T;


extern APP_DATA_T app_data;

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is called when a mouse button(left, right or middle) status
 * changes
 */
extern void HandleMousePioStatusChange(void);

/* This function is called when the pairing removal button is pressed for 1s */
extern void HandlePairingButtonPress(timer_id tid);

#endif /* _MOUSE_H_ */
