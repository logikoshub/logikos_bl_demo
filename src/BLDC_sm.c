/**
  ******************************************************************************
  * @file BLDC.c
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "bldc_sm.h"
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "driver.h"

/* Private defines -----------------------------------------------------------*/

//#define V_SHUTDOWN_THR      0x0368 // experimental  ...startup stalls are still possible!
#define V_SHUTDOWN_THR      0x0230 // tmp .. problems syncing the  phase    voltage measuremeht with PWM on-sector  ?


#define PWM_0PCNT      0

#define PWM_10PCNT     ( PWM_100PCNT / 10 )
#define PWM_20PCNT     ( PWM_100PCNT / 5 )
#define PWM_50PCNT     ( PWM_100PCNT / 2 )

#define PWM_X_PCNT( _PCNT_ )   ( _PCNT_ * PWM_100PCNT / 100 )

/*
 * precision is 1/TIM2_PWM_PD = 0.4% per count
 */
//#define PWM_DC_IDLE    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0
//#define PWM_DC_RAMPUP  PWM_DC_IDLE // PWM_X_PCNT( 14.0 )

#define PWM_DC_RAMPUP    PWM_X_PCNT( 12.0 )  // 0x1E ... 30 * 0.4 = 12.0

#define PWM_DC_IDLE      PWM_X_PCNT( PWM_DC_RAMPUP )

/*
 * These constants are the number of timer counts (TIM3) to achieve a given
 *  commutation step period.
 * See TIM3 setup - base period is 0.000000250 seconds (0.25 usec) in order to
 * provide high precision for controlling the commutation time, and each commutation step
 * unit is 4x TIM3 periods for back-EMF sampling at 1/4 and 3/4 in the commutation period.
 *
 * For the theoretical 1100kv motor @ 13.8v -> ~15000 RPM:
 *   15000 / 60 = 250 rps
 *   "Electrical cycles" per sec = 250 * (12/2) = 1500 ... where (12/2) is nr. of pole-pairs.
 *   Time of 1 cycle = 1/1500 = 0.000667 seconds  (360 degrees of 1 electrical cycle)
 *
 *   1 commutation sector is 60 degrees.
 *   Using TIM3 to get 4 updates per sector, and 360/15degrees=24 so ..
 *
 *   0.000667 seconds / 24 = 0.00002778 sec  (the "1/4 sector time" is 27.78us )
 *   ... divided by TIM3 base period (0.25 us)  -> 111 counts
 */

//#define TIM3_RATE_MODULUS   4 // each commutation sector of 60-degrees spans 4x TIM3 periods
// the commutation timing constants (TIM3 period) effectively have a factor of
// 'TIM3_RATE_MODULUS' rolled into them since the timer fires 4x faster than the
// actual motor commutation frequency.
#define BLDC_OL_TM_LO_SPD       0x1000 // 4096d  // start of ramp

//#define BLDC_OL_TM_HI_SPD       0x03C0 //  960d

//   0.000667 seconds / 24 / 0.25us = 111 counts
#define LUDICROUS_SPEED         0x006F // 111


/*
 * Slope of what is basically a linear startup ramp, commutation time (i.e. TIM3)
 * period) decremented by fixed amount each control-loop timestep. Slope
 * determined by experiment (conservative to avoid stalling the motor!)
 */
#define BLDC_ONE_RAMP_UNIT      2


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

//static  tmp
uint16_t Vsystem;


/* Private variables ---------------------------------------------------------*/

static BLDC_STATE_T BLDC_State;

static uint16_t BLDC_OL_comm_tm;

static uint16_t Commanded_Dutycycle; // PWM duty-cycle has to be ramped to this

static int vsys_fault_bucket;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief     .
 *   Simple ramping of commutation time period. At each iteration the
 *   commutation time period is ramped to the target value stepped in increment
 *   of +/- step depending of the sign of the error.
 *
 * @param   tgt_commutation_per  Target value to track.
 * @param   step integer step    Increment of the ramp (the slope).
 *
 * @return  +1 if positive increment
 *          -1 if negative increment
 *          0 if control variable equal to target
 */
int timing_ramp_control(uint16_t tgt_commutation_per, int increment)
{
    int error;
    int ret = 0;
    uint16_t u16 = get_commutation_period();

    error = tgt_commutation_per - u16;

    // determine signage of error i.e. step increment
    if (error < 0)
    {
        // negate the step parameter
        u16 -= increment;
        ret = -1;
    }
    else if (error > 0)
    {
        u16 += increment;
        ret = 1;
    }

    set_commutation_period( u16 );

    return ret;
}

/* Public functions ---------------------------------------------------------*/

/*
 * low-level stop: turns off all PWM
 */
void BLDC_Stop(void)
{
// kill the driver signals
    All_phase_stop();

    set_bldc_state( BLDC_OFF );
}

/*
 * increment set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Plus()
{
    if ( BLDC_OFF == get_bldc_state() )
    {
        set_bldc_state( BLDC_RAMPUP );
        return 0;
    }
    else if ( BLDC_ON == get_bldc_state() )
    {
//if (DC < PWM_DC_RAMPUP)
        inc_dutycycle();
    }
    return 0;
}

/*
 * decrement set and return present motor speed value
 */
uint16_t BLDC_PWMDC_Minus()
{
    if ( BLDC_OFF == get_bldc_state() )
    {
//        uart_print( "OFF->RAMP-\r\n");
        set_bldc_state(  BLDC_RAMPUP );
        return 0;
    }
    else if ( BLDC_ON == get_bldc_state() )
    {
// if (DC > PWM_20PCNT)
        dec_dutycycle();
    }
    return 0;
}

/*
 * sets motor speed from commanded throttle/UI setting  (experimental)
 */
void BLDC_PWMDC_Set(uint16_t dc)
{
    Commanded_Dutycycle = dc;
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_dec()
{
#if 1 // #ifdef DEBUG
    if ( BLDC_OFF == get_bldc_state() )
    {
        set_bldc_state( BLDC_RAMPUP );
    }

    if (BLDC_ON == get_bldc_state() /* && BLDC_OL_comm_tm < 0xFFFF */)
    {
        BLDC_OL_comm_tm += 1; // slower
    }
#endif
}

/*
 * TEST DEV ONLY: manual adjustment of commutation cycle time)
 */
void BLDC_Spd_inc()
{
#if 1 // #ifdef DEBUG
    if ( BLDC_OFF == get_bldc_state() )
    {
        set_bldc_state( BLDC_RAMPUP );
    }

    if (BLDC_ON == get_bldc_state() /* && BLDC_OL_comm_tm > BLDC_OL_TM_MANUAL_HI_LIM */ )
    {
        BLDC_OL_comm_tm -= 1; // faster
    }
#endif
}


/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
void set_commutation_period(uint16_t u16pd)
{
    BLDC_OL_comm_tm = u16pd;
}

uint16_t get_commutation_period(void)
{
    return BLDC_OL_comm_tm;
}

BLDC_STATE_T get_bldc_state(void)
{
    return BLDC_State;
}

BLDC_STATE_T set_bldc_state( BLDC_STATE_T newstate)
{
    BLDC_State = newstate;
}

/*
 * BLDC Update:
 *  Called from ISR
 *  Handle the BLDC state:
 *      Off: nothing
 *      Rampup: get BLDC up to sync speed to est. comm. sync.
 *              Once the HI OL speed (frequency) is reached, then the idle speed
 *              must be established, i.e. controlling PWM DC to ? to achieve 2500RPM
 *              To do this closed loop, will need to internally time between the
 *              A/D or comparator input interrupts and adjust DC using e.g. Proportional
 *              control. When idle speed is reached, can transition to user control i.e. ON State
 *      On:  definition of ON state - user control (button inputs) has been enabled
 *              1) ideally, does nothing - BLDC_Step triggered by A/D comparator event
 *              2) less ideal, has to check A/D or comp. result and do the comm.
 *                 step ... but the resolution will be these discrete steps
 *                 (of TIM1 reference)
 */
void BLDC_Update(void)
{
    const int FAULT_BUCKET_INI = 128;

    // some actions only done on state transitions (prev state must be updated at end of this function)
    static BLDC_STATE_T prev_bldc_state = BLDC_OFF;

// if the voltage threshold is high enuff, the ramp delay time thingy not needed
//    const int RAMP_TIME = 1;   // fault arming delay time
//    static uint16_t fault_arming_time; // fault_arming_time

    int itemp;

    BLDC_STATE_T bldc_state = get_bldc_state();

    switch ( bldc_state )
    {
    default:
    case BLDC_OFF:
        // reset commutation timer and ramp-up counters ready for ramp-up
        set_commutation_period( BLDC_OL_TM_LO_SPD );

        vsys_fault_bucket = FAULT_BUCKET_INI;

        // delay to wait to stabillize at first DC setpoint post-ramp
//        fault_arming_time = RAMP_TIME;    /////// // reset the static ramp timer

        set_dutycycle( PWM_0PCNT );
        break;

    case BLDC_ON:
        // do ON stuff
        Vsystem = Driver_Get_Vbatt() / 2 + Vsystem / 2; // sma

#if 0  // .. Todo: needs to adjust threshold for in-ramp
        if ( fault_arming_time  > 0 )
        {
            fault_arming_time   -= 1;
        }
        else    // assert (Vbatt > VVV )
#endif
        {
            // check system voltage ... is motor stalled?
            if (Vsystem < V_SHUTDOWN_THR)
            {
                // voltage has sagged ... likely motor stall!
                if ( vsys_fault_bucket  > 0)
                {
                    vsys_fault_bucket -= 1; //
                }
            }
            else
            {
                if ( vsys_fault_bucket  < FAULT_BUCKET_INI )
                {
                    vsys_fault_bucket += 1; // refillin leaky bucket
                }
            }
// finally, check if fault is set
            if (0 == vsys_fault_bucket)
            {
#if 1 // #if ENABLE_VLOW_FAULT
                // 0 DC safely stops the motor, user must still press STOP to cycle the program.
                set_dutycycle( PWM_0PCNT );
#endif
            }
        }

//        if ( 0 == get_op_mode() )
        {
            const int step = 1;
            timing_ramp_control( Get_OL_Timing( get_dutycycle() ), step );
        }
        break;

    case BLDC_RAMPUP:

        if (BLDC_RAMPUP != prev_bldc_state)
        {
            // set the ramp DC upon transition into ramp state
            set_dutycycle( PWM_DC_RAMPUP );
        }

        itemp = timing_ramp_control(
            Get_OL_Timing( /* PWM_DC_RAMPUP */ get_dutycycle() ), BLDC_ONE_RAMP_UNIT );

        if ( itemp >= 0 ) // ( comm_time < target )
        {
            // state-transition trigger
            set_bldc_state( BLDC_ON );

            // set initial condition of filtered system voltage masurement
            Vsystem = 0x0400;
        }
        break;
    }

    prev_bldc_state = bldc_state ;

//   the timer for the OL commutation switch time must be updated upon return (setup Timer 3)
}
