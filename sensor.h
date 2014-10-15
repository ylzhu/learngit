/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      sensor.h
 *
 *  DESCRIPTION
 *      This file contains definitions for the Mouse sensor routines. The sensor
 *      used for this application is Avago ADNS-7530.
 *
 ******************************************************************************/

#ifndef _SENSOR_H_
#define _SENSOR_H_

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <time.h>

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/*
 * SPI PIO definitions for sensor
 */
#define SENSOR_SPI_MOSI_PIO                 (3)
#define SENSOR_SPI_MIS0_PIO                 (4)
#define SENSOR_SPI_CLK_PIO                  (0)
#define SENSOR_SPI_NCS_PIO                  (1)

/*
 * Laser current configuration values
 */
#define SENSOR_LASER_CURRENT_1              (0)     /* 0.9..3mA */
#define SENSOR_LASER_CURRENT_2              (1)     /* 2..5mA */
#define SENSOR_LASER_CURRENT_3              (3)     /* 4..10mA */

/*
 * Relative laser power adjustments
 */
#define SENSOR_LASER_MINIMUM_POWER          (0x00)
#define SENSOR_LASER_FULL_POWER             (0xFF)

/* Sensor frame rates and downshift times */

/* Default configuration for the run mode frame rate: 8ms. This value is
 * configured to be the highest value allowed by the sensor to reduce
 * power consumption.
 */
#define SENSOR_MAX_RUN_FRAMERATE            (8)
/* Run to Rest1 mode downshift: 128ms */
#define SENSOR_RUN_DOWNSHIFT                (128)
/* Rest1 mode frame rate: 20ms */
#define SENSOR_REST1_FRAMERATE              (20)
/* Rest1 to Rest2 mode downshift: 5s */
#define SENSOR_REST1_DOWNSHIFT              (5000)
/* Rest2 mode frame rate: 100ms */
#define SENSOR_REST2_FRAMERATE              (100)
/* Rest2 to Rest3 mode downshift: 5min */
#define SENSOR_REST2_DOWNSHIFT              (300000)
/* Rest3 mode frame rate: 0.5s */
#define SENSOR_REST3_FRAMERATE              (500)

/*=============================================================================*
 *  Public Data Types
 *============================================================================*/

/* Sensor run modes */
typedef enum
{
    sensor_mode_run                 = 0,
    sensor_mode_rest1               = 1,
    sensor_mode_rest2               = 2,
    sensor_mode_rest3               = 3
} sensor_run_modes;

/* Different sensor resolution values as specified in bit 5 and 6 of 
 * configuration register(0x12)
 */
typedef enum
{
    sensor_resolution_400_cpi       = 0,
    sensor_resolution_800_cpi       = 1,
    sensor_resolution_1200_cpi      = 2,
    sensor_resolution_1600_cpi      = 3
} sensor_resolution;

/*=============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function initialises the sensor */
extern bool SensorInit(uint8 laserCurrent, uint8 relLaserPower);

/* This function configures the run rate, rest1, rest2 and rest3 frame rates and
 * the time between transition to different states
 */
extern void SensorConfigureModes(void);

/* This function burst reads the dXY values from the sensor */
extern bool SensorReadReport(uint16* dx, uint16* dy, uint8* status);

/* This function updates the rate at which the sensor updates its registers in
 * run mode.
 */
extern void SensorUpdateFrameRate(uint16 conn_interval);

/* This function writes to motion register of the sensor */
extern void WriteToMotionReg(void);

/* This function gets the present mouse resolution by reading from the sensor
 * register
 */
extern uint8 GetSensorResolution(void);

/* This function sets the mouse resolution to the given value by writing to
 * the sensor register.
 */
extern bool SetSensorResolution(sensor_resolution resolution);

#endif /* _SENSOR_H_ */
