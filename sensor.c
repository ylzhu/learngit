/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2014
 *  Part of CSR uEnergy SDK 2.3.0
 *  Application version 2.3.0.0
 *
 *  FILE
 *      sensor.c
 *
 *  DESCRIPTION
 *      This file contains implementation for the Mouse sensor routines. The
 *      sensor used for this application is Avago ADNS-7530.
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <time.h>
#include <spi.h>
                                                  
/*=============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "sensor.h"

/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

uint8 mouseRes = 0;

/* SPI read/write routines */
#define SENSOR_WRITE_REG(reg, val)      SpiWriteRegister(reg | 0x80, val)   
/* ADNS sensor needs first bit of address set for register write */
#define SENSOR_READ_REG(reg)            SpiReadRegister(reg)
#define SENSOR_BURST_READ(reg, dest, n) \
                                     SpiReadRegisterBurst(reg, dest, n, FALSE);

/* Avago ADNS-7530 registers */
/* Delay between two consequent write commands */
#define SENSOR_WRITE_WRITE_DELAY        (30)  /* 30 micro-seconds */
/* Delay between consequent write and read commands */
#define SENSOR_WRITE_READ_DELAY         (20)  /* 20 micro-seconds */
/* Delay between consequent read commands */
#define SENSOR_READ_READ_DELAY          (1)   /* 1 micro-second */

#define SENSOR_MOTION_REG               (0x02) /* Motion */
#define SENSOR_DELTA_XL_REG             (0x03) /* Delta X, lower 8 bits */
#define SENSOR_DELTA_YL_REG             (0x04) /* Delta Y, lower 8 bits */
#define SENSOR_DELTA_XYH_REG            (0x05) /* Delta XY, high 4 bits */
#define SENSOR_CONFIGURATION2_BITS_REG  (0x12) /* Laser sensor configuration */
#define SENSOR_RUN_DOWNSHIFT_REG        (0x13) /* Run->Rest 1 downshift time */
#define SENSOR_REST1_RATE_REG           (0x14) /* Rest 1 frame rate */
#define SENSOR_REST1_DOWNSHIFT_REG      (0x15) /* Rest 1->Rest 2 downshift time
                                                */
#define SENSOR_REST2_RATE_REG           (0x16) /* Rest 2 frame rate */
#define SENSOR_REST2_DOWNSHIFT_REG      (0x17) /* Rest 2->Rest 3 downshift time
                                                */
#define SENSOR_REST3_RATE_REG           (0x18) /* Rest 3 frame rate */
#define SENSOR_LASER_CTRL0_REG          (0x1A) /* Laser control register */
#define SENSOR_LASER_CTRL1_REG          (0x1F) /* Laser control complement
                                                * register
                                                */
#define SENSOR_LSRPWR_CFG0_REG          (0x1C) /* Laser current register */
#define SENSOR_LSRPWR_CFG1_REG          (0x1D) /* Laser current complement
                                                * register
                                                */
#define SENSOR_OBSERVATION_REG          (0x2E) /* Observation register */
#define SENSOR_RESET_REG                (0x3A) /* Reset register */
#define SENSOR_MOTION_BURST_REG         (0x42) /* Motion burst register */

#define SENSOR_RESET_VALUE              (0x5A) /* Reset value */
                                                 
#define SENSOR_MOTION_DATA_AVAILABLE    (0x80) /* Bitmask for valid data */

/* Sensor timeouts */
#define SENSOR_TIME_ONE_FRAME           (1*MILLISECOND) /* One frame at 30 ips,
                                                         * 800 cpi
                                                         */
#define SENSOR_RESET_TIME               (23*MILLISECOND)/* Reset delay */

/* The connection interval in 'number of frames' above which the run rate to be
 * used for the sensor is 8 ms.
 */
#define CONNECTION_INTERVAL_LIMIT       (9)

/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static bool SensorConfigureRestMode(sensor_run_modes mode,
                                         uint16 modeRate, uint32 modeDownshift);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *    SensorConfigureRestMode
 * 
 *  FUNCTION
 *    Configures rate and downshift time for the different mouse sensor modes
 *    mode            - mouse mode (run, rest 1/2/3)
 *    modeRate        - frame rate in ms, 2..8ms for Run mode, 20..2410ms for
 *                      the Rest modes
 *    modeDownshift   - downshift time in ms when sensor switch to the lower
 *                      power mode Run > Rest 1 > Rest 2 > Rest 3. Ignored for
 *                      Rest 3.
 *
 *  RETURNS
 *    TRUE for successful attempt, FALSE for incorrect parameters
 *----------------------------------------------------------------------------*/
static bool SensorConfigureRestMode(sensor_run_modes mode, uint16 modeRate,
                                                          uint32 modeDownshift)
{
    uint8 frameRateReg = SENSOR_REST1_RATE_REG;
    uint8 dwnshiftReg = SENSOR_RUN_DOWNSHIFT_REG;
    uint8 dwnshiftTime = 0;
    
    switch(mode)
    {
        case sensor_mode_run:
            /* Run mode */
            dwnshiftTime = modeDownshift / (8UL * (uint32)modeRate);
            
        break;
        case sensor_mode_rest1:
            /* Rest 1 mode */
            frameRateReg = SENSOR_REST1_RATE_REG;
            dwnshiftReg = SENSOR_REST1_DOWNSHIFT_REG;
            
            dwnshiftTime = (modeDownshift) / (16UL * (uint32)modeRate);
            
        break;
        case sensor_mode_rest2:
            /* Rest 2 mode */
            frameRateReg = SENSOR_REST2_RATE_REG;
            dwnshiftReg = SENSOR_REST2_DOWNSHIFT_REG;
            
            dwnshiftTime = (modeDownshift) / (128UL * (uint32)modeRate);
            
        break;
        case sensor_mode_rest3:
            /* Rest 3 mode */
            frameRateReg = SENSOR_REST3_RATE_REG;
            
        break;
        default:
            return FALSE;
            
        break;
    }
    
    /* Set the frame rate */
    if(mode == sensor_mode_run)
    {
        /* Special care for Run mode */
        uint8 config2Reg;
        uint8 value;

        /* The valid values for run_rate are 2 to 8. If 'modeRate' is invalid,
         * return without writing it to the configuration register. For more
         * information, refer to 'Configuration2_bits' section of datasheet.
         */
        if((modeRate < 2) || (modeRate > 8))
            return FALSE;
        
        value = modeRate - 2;
        
        /* Configuration register also has AWAKE, resolution and reserved fields
         * Preserve these bits as they are and write only to the run_rate bits
         * of the register. For more information, refer to 'Configuration2_bits'
         * section of datasheet.
         */
        config2Reg = SENSOR_READ_REG(SENSOR_CONFIGURATION2_BITS_REG);
        TimeDelayUSec(SENSOR_READ_READ_DELAY);
        config2Reg = (config2Reg & 0xf8) | value;
        SENSOR_WRITE_REG(SENSOR_CONFIGURATION2_BITS_REG, config2Reg);
    }
    else
    {
        uint8 value;

        /* The valid values of frame rate for other modes are from 20 ms to
         * 2.41 seconds. For more information, refer to Rest1_Rate, Rest2_Rate
         * and Rest3_Rate registers' documentation in datasheet.
         */
        if((modeRate < 20) || (modeRate > 2410))
            return FALSE;
        
        value = modeRate / 10 - 1;
        SENSOR_WRITE_REG(frameRateReg, value);
    }
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);
    
    /* Set the downshift time */
    if(mode != sensor_mode_rest3)
    {
        if((dwnshiftTime < 1) || (dwnshiftTime > 242))
            return FALSE;

        SENSOR_WRITE_REG(dwnshiftReg, dwnshiftTime);
        TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);
    }
    
    return TRUE;
}

/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *    SensorInit
 *
 *  DESCRIPTION
 *    This function initialises the sensor.
 *    
 * PARAMETERS
 *    laserCurrent    - laser current level (use one of the
 *                                           SENSOR_LASER_CURRENT_x constants)
 *    relLaserPower   - laser current level adjustments, 1 step is (1/384)*100%
 *                      = 0.26% drop of relative laser current. E.g. value of
 *                      0 = 33.59% of laser power, 0xFF = 100%
 *
 * RETURNS
 *    TRUE in case of success
 *----------------------------------------------------------------------------*/

extern bool SensorInit(uint8 laserCurrent, uint8 relLaserPower)
{
    uint8 observeReg;
    
    /* Initialize SPI */
    SpiInit(SENSOR_SPI_MOSI_PIO, SENSOR_SPI_MIS0_PIO, SENSOR_SPI_CLK_PIO, 
                                                            SENSOR_SPI_NCS_PIO);
    
    /* Power-up procedure as in sensor datasheet */
    
    /* Assert reset */
    SENSOR_WRITE_REG(SENSOR_RESET_REG, SENSOR_RESET_VALUE);
    TimeDelayUSec(SENSOR_TIME_ONE_FRAME);

    /* Clear observation register and re-read */
    SENSOR_WRITE_REG(SENSOR_OBSERVATION_REG, 0x00);
    TimeDelayUSec(5*SENSOR_TIME_ONE_FRAME);
    observeReg = SENSOR_READ_REG(SENSOR_OBSERVATION_REG);
    
    if(!(observeReg & 0x1F))
    {
        /* 4 LSB should be set high */
        return FALSE;
    }
        
    /* Read from register 0x02..0x05 */
    SENSOR_READ_REG(SENSOR_MOTION_REG);
    SENSOR_READ_REG(SENSOR_DELTA_XL_REG);
    SENSOR_READ_REG(SENSOR_DELTA_YL_REG);
    SENSOR_READ_REG(SENSOR_DELTA_XYH_REG);
    
    /* Finish the initialization. Refer to 'Notes on power up' section of
     * datasheet.
     */
    SENSOR_WRITE_REG(0x3C, 0x27);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);
    
    SENSOR_WRITE_REG(0x22, 0x0A);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    SENSOR_WRITE_REG(0x21, 0x01);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    SENSOR_WRITE_REG(0x3C, 0x32);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    SENSOR_WRITE_REG(0x23, 0x20);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    SENSOR_WRITE_REG(0x3C, 0x05);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    SENSOR_WRITE_REG(0x37, 0xB9);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);
    
    /* Set the laser power level. Refer to 'Laser power adjustment procedure'
     * section of datasheet.
     */

    /* Write current configuration to the LASER_CTRL0 register */
    SENSOR_WRITE_REG(SENSOR_LASER_CTRL0_REG, laserCurrent & 0xC0);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);
    
    /* Write complement of the current to the LASER_CTRL1 register */
    SENSOR_WRITE_REG(SENSOR_LASER_CTRL1_REG, (~laserCurrent) & 0xC0);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    /* Write laser current adjustments to LSRPWR_CFG0 register */
    SENSOR_WRITE_REG(SENSOR_LSRPWR_CFG0_REG, 0xFF);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    /* Write complement of the current adjustments to LSRPWR_CFG1 register */
    SENSOR_WRITE_REG(SENSOR_LSRPWR_CFG1_REG, 0x00);
    TimeDelayUSec(SENSOR_WRITE_WRITE_DELAY);

    return TRUE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      SensorConfigureModes
 *
 *  DESCRIPTION
 *      This function is called to configure the run rate, rest1, rest2 and
 *      rest3 frame rates as well as the time between transition to different
 *      states.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
extern void SensorConfigureModes(void)
{
    /* Set the run rate and downshift time to rest1 state */
    SensorConfigureRestMode(sensor_mode_run, SENSOR_MAX_RUN_FRAMERATE,
                                             SENSOR_RUN_DOWNSHIFT);
    /* Set the rest1 frame rate and downshift time to rest2 state */
    SensorConfigureRestMode(sensor_mode_rest1, SENSOR_REST1_FRAMERATE, 
                                               SENSOR_REST1_DOWNSHIFT);
    /* Set the rest2 frame rate and downshift time to rest3 state */
    SensorConfigureRestMode(sensor_mode_rest2, SENSOR_REST2_FRAMERATE,
                                               SENSOR_REST2_DOWNSHIFT);
    /* Set the rest3 frame rate */
    SensorConfigureRestMode(sensor_mode_rest3, SENSOR_REST3_FRAMERATE, 0);
    
}

/*-----------------------------------------------------------------------------*
 * NAME
 *    SensorReadReport
 * 
 * FUNCTION
 *    This function burst reads the dXY values from the sensor
 *
 * RETURNS
 *    TRUE when there is valid data, FALSE otherwise
 *----------------------------------------------------------------------------*/
extern bool SensorReadReport(uint16* dx, uint16* dy, uint8* status)
{
    uint8 data[4];
    
    /* Burst read Motion, Delta_X_L, Delta_Y_L and Delta_XY_H registers */
    SENSOR_BURST_READ(SENSOR_MOTION_BURST_REG, data, 4);
    
    /* Store status */
    *status = data[0];
    
    if(data[0] & SENSOR_MOTION_DATA_AVAILABLE)
    {
        /* delta-XY data available, assemble 12-bit 2's complements */
        *dx = ((uint16)(data[3] & 0xF0) << 4) | ((uint16)data[1] & 0x00FF);
        *dy = ((uint16)(data[3] & 0x0F) << 8) | ((uint16)data[2] & 0x00FF);
        
        return TRUE;
    }
    
    *dx = 0;
    *dy = 0;

    return FALSE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *    SensorUpdateFrameRate
 * 
 * FUNCTION
 *    This function updates the rate at which the sensor updates its motion
 *    sensing registers in run mode.
 *    Connection interval(number of frames of 1.25ms)    Run rate to be used(ms)
 *                        6                                       4
 *                        7                                       5
 *                        8                                       6
 *                        9                                       7
 *                        10 and above                            8
 *
 * RETURNS
 *    Nothing.
 *----------------------------------------------------------------------------*/
extern void SensorUpdateFrameRate(uint16 conn_interval)
{
    uint16 run_rate;
    /* If the connection interval is less than the connection interval limit,
     * then the run frame rate is variably configured to make sure that
     * new data is updated in sensor registers once in every connection
     * interval.
     */
    if(conn_interval <= CONNECTION_INTERVAL_LIMIT)
    {
        run_rate = conn_interval - 2;
    }
    /* Otherwise, set the run frame rate to the highest possible value of the
     * sensor.
     */
    else
    {
        run_rate = SENSOR_MAX_RUN_FRAMERATE;
    }
    
    /* Set the run rate and downshift time to rest1 state */
    SensorConfigureRestMode(sensor_mode_run, run_rate, SENSOR_RUN_DOWNSHIFT);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *    WriteToMotionReg
 * 
 * FUNCTION
 *    This function writes to motion register. As per the data sheet of mouse,
 *    writing to motion register, clears the MOT and OVF bits, Delta_X_L,
 *    Delta_Y_L and Delta_XY_H registers.
 *
 * RETURNS
 *    TRUE when there is valid data, FALSE otherwise
 *----------------------------------------------------------------------------*/
extern void WriteToMotionReg()
{
    SENSOR_WRITE_REG(SENSOR_MOTION_REG, 0x00);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *    GetSensorResolution
 * 
 * FUNCTION
 *    This function finds the present mouse resolution by reading from the
 *    sensor register.
 *
 * RETURNS
 *    Resolution value as mapped below:
 *    00 = 400
 *    01 = 800
 *    10 = 1200
 *    11 = 1600
 *    that is, returns 0 if the resolution is 400 CPI(Counts Per Inch).
 *----------------------------------------------------------------------------*/
extern uint8 GetSensorResolution(void)
{
    sensor_resolution resolution;

    /* Resolution is stored in bits 5 and 6 of configuration register(0x12) */
    resolution = ((SENSOR_READ_REG(SENSOR_CONFIGURATION2_BITS_REG)) & 
                                                                 (0x60)) >> 5;
    return (uint8)resolution;
}

/*-----------------------------------------------------------------------------*
 * NAME
 *    SetSensorResolution
 * 
 * FUNCTION
 *    This function sets the mouse resolution to the given value by writing to
 *    the sensor register.
 *
 * RETURNS
 *    Boolean - Whether the value of the resolution to be set belongs to the set
 *    of values acceptable by the sensor(TRUE) or not(FALSE)
 *
 *----------------------------------------------------------------------------*/
extern bool SetSensorResolution(sensor_resolution resolution)
{
    bool valid_value = FALSE;
    
    /* Update the resolution only if the remote device has sent a valid value */
    if(resolution <= sensor_resolution_1600_cpi)
    {
        uint8 config2Reg;

        valid_value = TRUE;
        
        config2Reg = SENSOR_READ_REG(SENSOR_CONFIGURATION2_BITS_REG);
        TimeDelayUSec(SENSOR_READ_READ_DELAY);
        /* Resolution needs to be written to bits 5 and 6 of register 0x12.
         * So left shift the value 5 times before writing it to the sensor 
         * register.
         */
        config2Reg = (config2Reg & 0x9f) | (resolution << 5);
        SENSOR_WRITE_REG(SENSOR_CONFIGURATION2_BITS_REG, config2Reg);
    }

    return valid_value;
}
