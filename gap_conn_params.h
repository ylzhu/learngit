/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      gap_conn_params.h
 *
 *  DESCRIPTION
 *      MACROs for conn param update values
 *
 ******************************************************************************/

#ifndef _GAP_CONN_PARAMS_H_
#define _GAP_CONN_PARAMS_H_

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Idle timer value in Connected state. At the expiry of this timer,
 * the mouse will disconnect itself from the Host.
 */
#define CONNECTED_IDLE_TIMEOUT_VALUE          (30 * MINUTE)

/* Advertising parameters, time is expressed in microseconds and the firmware
 * will round this down to the nearest slot. Acceptable range is 20ms to 10.24s
 * and the minimum must be no larger than the maximum.This value needs to be 
 * modified at later stage as decided GPA for specific profile.
 *
 * To enable fast connections though the recommended range is between 20 ms to
 * 30 ms, but it has been observed that it is way too energy expensive. So, we
 * have decided to use 60 ms as the fast connection advertisement interval. For
 * reduced power connections, the recommended range is between 1s to 2.5 s.
 * Vendors will need to tune these values as per their requirements.
 */
#define FC_ADVERTISING_INTERVAL_MIN           (60 * MILLISECOND)
#define FC_ADVERTISING_INTERVAL_MAX           (60 * MILLISECOND)
    
#define RP_ADVERTISING_INTERVAL_MIN           (1280 * MILLISECOND)
#define RP_ADVERTISING_INTERVAL_MAX           (1280 * MILLISECOND)

#define FAST_CONNECTION_ADVERT_TIMEOUT_VALUE  (30 * SECOND)

/* Time for which mouse will trigger slow undirected advertisements. Since,
 * limited discoverable mode is being used by mouse when it is not bonded,
 * the maximum value of this macro can 30.72 seconds as specified in Bluetooth
 * Core Specification version 4.0, Section 16, Appendix A in the GAP
 * specification
 */

#define SLOW_CONNECTION_ADVERT_TIMEOUT_VALUE  (30 * SECOND)

/* Brackets should not be used around the value of macros that are used in .db
 * files. The parser which creates .c and .h files from .db file doesn't
 * understand  brackets and will raise syntax errors.
 */

/* Preferred connection parameters should be within the range specified in the 
 * Bluetooth specification.
 */
/* Minimum and maximum connection interval in number of frames */
#define PREFERRED_MAX_CON_INTERVAL            0x000a /* 12.5 ms */
#define PREFERRED_MIN_CON_INTERVAL            0x000a /* 12.5 ms */

/* Slave latency in number of connection intervals */
#define PREFERRED_SLAVE_LATENCY               0x0064 /* 100 conn_intervals. */

/* Supervision timeout (ms) = PREFERRED_SUPERVISION_TIMEOUT * 10 ms */
#define PREFERRED_SUPERVISION_TIMEOUT         0x0320 /* 8 seconds. */



/* APPLE Compliant connection parameters */
/* Minimum and maximum connection interval in number of frames. */
#define APPLE_MAX_CON_INTERVAL                0x0010 /* 20 ms */
#define APPLE_MIN_CON_INTERVAL                0x0020 /* 40 ms */

/* Slave latency in number of connection intervals. */
#define APPLE_SLAVE_LATENCY                   0x0004 /* 4 conn_intervals. */

/* Supervision timeout (ms) = PREFERRED_SUPERVISION_TIMEOUT * 10 ms */
#define APPLE_SUPERVISION_TIMEOUT             0x0258 /* 6 seconds */


/* Max num of conn param update that we send in one connection*/
#define MAX_NUM_CONN_PARAM_UPDATE_REQS        4

/* Number of times out of the total maximum number of connection parameter 
 * update attempts(MAX_NUM_CONN_PARAM_UPDATE_REQS) when the application requests
 * its own preferred connection paramaters.
 */
#define CPU_SELF_PARAMS_MAX_ATTEMPTS          2

/* Number of times out of the total maximum number of connection parameter 
 * update attempts(MAX_NUM_CONN_PARAM_UPDATE_REQS) when the application requests
 * its APPLE preferred connection paramaters.
 */
#define CPU_APPLE_PARAMS_MAX_ATTEMPTS         (MAX_NUM_CONN_PARAM_UPDATE_REQS -\
                                                  CPU_SELF_PARAMS_MAX_ATTEMPTS)

#endif /* _GAP_CONN_PARAMS_H_ */