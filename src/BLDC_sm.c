/**
  ******************************************************************************
  * @file BLDC_sm.c
  * @brief state-manager for BLDC
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
/**
 * \defgroup  BLDC_sm BLDC State
 * @brief  BLDC state management and timing control
 * @{
 */
/* Includes ------------------------------------------------------------------*/
#include "bldc_sm.h" // external types used internally
#include "mdata.h"
#include "pwm_stm8s.h" // motor phase control
#include "faultm.h"
#include "sequence.h"

/* Private defines -----------------------------------------------------------*/
/*
 * Commutation timing values at ramp end-points originated from a fixed
 * closed-loop timing table, hard-coded for 1100kv motor @ 12.v.
 * CTIME SCALAR is a vestige of rescaling the integer tables values which all
 * a common factor that being the system clock.
 */
// commutation period at start of ramp
#define BL_CT_RAMP_START  (5632.0 * CTIME_SCALAR) // $1600

// commutation period at end of ramp
#define BL_CT_RAMP_END    (1760.0 * CTIME_SCALAR) // $06E0

// for some reason this little slowdown at ramp end aids in getting sync (experimental/TBD)
#define BL_CT_STARTUP     (1866.0 * CTIME_SCALAR) // $074A

/*
 * Error limit used in BL_cl_control -
 * needs to be small enough to be stable upon transition from to closed-loop
 */
#define ERROR_LIMIT	50

/**
 * @brief Control rate scalar
 * @details Scale factor relating the commutation-timing ramp data and variables
 *     with the control task rate
 */
#define CTRL_RATEM  4.0

// The control-frame rate becomes factored into the integer ramp-step
#define BL_ONE_RAMP_UNIT  (1.125 * CTRL_RATEM * CTIME_SCALAR) // GN: 07-JUN-22 BL-21.4 @13.0v needed slower rampup

// length of alignment step (experimentally determined w/ 1100kv @12.5v)
#define BL_TIME_ALIGN  (200 * 1) // N frames @ 1 ms / frame


// sets the rate of change of the commanded motor speed
#define BL_SPEED_RAMP_STEP    (PWM_PERIOD_COUNTS / 250)


/* Private types -----------------------------------------------------------*/

/* Public variables  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t BL_comm_period; // persistent value of ramp timing
static uint16_t BL_motor_speed; // persistent value of motor speed
static uint16_t BL_optimer; // allows for timed op state (e.g. alignment)
static BL_State_T BL_opstate; // BL operation state

/* Private function prototypes -----------------------------------------------*/
static bool BL_cl_control(uint16_t current_setpoint);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Commutation timing ramp control.
 *
 * At each iteration the commutation time period is ramped to the target value
 * stepped in increment of +/- step depending on the sign of the error.
 *
 * @param setpoint Target value to track.
 * @param target Target value to track.
 */
static void timing_ramp_control(uint16_t current_setpoint, uint16_t target_setpoint)
{
  uint16_t u16 = current_setpoint;

  // determine signage of error i.e. step increment
  if (u16 > target_setpoint)
  {
    u16 -= (uint16_t)BL_ONE_RAMP_UNIT;
    if (u16 < target_setpoint)
    {
      u16 = target_setpoint;
    }
  }
  else if (u16 < target_setpoint)
  {
    u16 += (uint16_t)BL_ONE_RAMP_UNIT;
    if (u16 > target_setpoint)
    {
      u16 = target_setpoint;
    }
  }
  BL_set_timing(u16);
}

/**
 * @Brief common sub for stopping and fault states
 *
 * @Detail
 * Allows motor to be stopped in a fault condition, leaving the system to
 * remain in whatever operating state - does not reset the control state, fault
 * manageer etc. This is a developers "feature" allowing the fault state and
 * other info to be examined.
 */
static void BL_stop(void)
{
  // kill the driver signals
  All_phase_stop();

  // have to clear the local UI_speed since that is the transition OFF->RAMP condition
  BL_motor_speed = 0;
}

/* Public functions ---------------------------------------------------------*/
/**
 * @brief Initialize/reset motor
 *
 *    System reset / re-arm function (has to be called both at program startup
 *    as well as following a fault condition state.
 *
 * @details
 *    expect to be called from non-ISR/CS context (i.e. from  UI handler)
 */
void BL_reset(void)
{
  // assert PWM channels to disabled
  BL_stop();

  // Set initial commutation timing period upon state transition.
  // TIM3 is left enabled, so the commutation period (TIM3) is simply set to a
  // arbitrarily large number. The TIM3 ISR will still fire but the commutation
  // step logic has no effect as long as the PWM is disabled.
  BL_set_timing( U16_MAX ); // 0xFFFF;

  Faultm_init();

  BL_set_opstate( BL_STOPPED ); // set the initial control-state
}

/**
 * @brief Sets motor speed from commanded throttle/UI setting
 *
 * @details
 *  The motor is started once reaching the ramp speed threshold, and allowed to
 *  slow down to the low shutoff threshold.
 *  UI Speed is shared with background task so this function should
 *  be invoked only from within a CS.
 *
 * @param ui_mspeed_counts The desired motor output in terms of timer counts
 */
void BL_set_speed(uint16_t ui_mspeed_counts)
{
  if (ui_mspeed_counts > PWM_PD_SHUTOFF)
  {
    // Update the dc if speed input greater than ramp start, OR if system already running
    if ((ui_mspeed_counts > PWM_PD_STARTUP) || (0 != BL_motor_speed))
    {
      BL_motor_speed = ui_mspeed_counts;
    }
  }
  else // if (ui_mspeed_counts <= PD_SHUTOFF)
  {
    // allow everything to reset once the throttle is lowered
    BL_reset();
  }
}

/**
 * @brief Accessor for Commanded Duty Cycle
 *
 * @return PWM period
 */
uint16_t BL_get_speed(void)
{
  return BL_motor_speed;
}

/**
 * @brief adjust commutation timing by step amount
 */
void BL_timing_step_slower()
{
  BL_set_timing(BL_comm_period + (uint16_t)BL_ONE_RAMP_UNIT);
}
/**
 * @brief adjust commutation timing by step amount
 */
void BL_timing_step_faster()
{
  BL_set_timing(BL_comm_period - (uint16_t)BL_ONE_RAMP_UNIT);
}

/**
  * @brief Accessor for commutation period.
  *
  * @return commutation period
  */
uint16_t BL_get_timing(void)
{
  return BL_comm_period;
}

/**
  * @brief Accessor for commutation period.
  */
void BL_set_timing(uint16_t u16)
{
  BL_comm_period = u16;
}

/**
 * @brief Accessor for state variable.
 *
 * @details
 *  External modules can query if the machine is running or not.
 *  There are only these two states bases on the set speed greater or less than
 *  the shutdown threshold.
 *
 * @return state value
 */
BL_RUNSTATE_t BL_get_state(void)
{
  if ( BL_motor_speed > PWM_PD_SHUTOFF )
  {
    return BL_IS_RUNNING;
  }
  // else
  return BL_NOT_RUNNING;
}

/**
 * @brief  Accessor for state variable
 */
void BL_set_opstate(uint8_t opstate)
{
  BL_opstate = opstate;
}

/**
 * @brief  Accessor for state variable
 *
 * @return  operation state
 */
uint8_t BL_get_opstate(void)
{
  return BL_opstate;
}

/**
 * @ brief closed loop controller
 * @ return boolean true (success) false (fail)
 */
/**
 * @brief closed loop control function
 * @param current_setpoint commutation period
 * @return TRUE: within control limits, FALSE: not within control limits
 */
static bool BL_cl_control(uint16_t current_setpoint)
{
  // returns true if plausible conditions for transition to closed-loop
  if (FALSE != Seq_get_timing_error_p())
  {
    // needs to be small enough to be stable upon transition from to closed-loop
    static const int16_t ERROR_MAX = ERROR_LIMIT;
    static const int16_t ERROR_MIN = -1 * ERROR_LIMIT;
    int16_t timing_error = Seq_get_timing_error();

    if ((timing_error > ERROR_MIN) && (timing_error < ERROR_MAX))
    {
      static const int16_t PROP_GAIN = 10; // inverse of kP
      int16_t correction = timing_error / PROP_GAIN ;

      BL_set_timing(current_setpoint + correction);

      return TRUE;
    }
  }
  return FALSE;
}

/**
 * @brief step the motor speed from the present operation point toward the input speed setpoint
 */
uint16_t bl_get_ramped_speed(uint16_t input_speed)
{
  // get the presently set speed/duty-cycle as default
  uint16_t ramped_speed = PWM_get_dutycycle();

  if (ramped_speed < input_speed)
  {
    ramped_speed += 1u; // +BL_SPEED_RAMP_STEP ... TODO? use fixed-point macro and make range-check robust
  }
  else if (ramped_speed > input_speed)
  {
    ramped_speed -= 1u; // -BL_SPEED_RAMP_STEP ... TODO? use fixed-point macro and make range-check robust
  }
  return ramped_speed;
}

/**
 * @brief  Implement control task (fixed exec rate of ~1ms).
 */
void BL_State_Ctrl(void)
{
  uint16_t inp_dutycycle = 0; // in case of error, PWM output remains 0

  if ( 0 != Faultm_get_status() )
  {
    // sets BL pwm period to 0 and disables timer PWM channels but doesn't
    // re-init the system state
    BL_stop();
  }
  else
  {
    BL_State_T bl_opstate = BL_get_opstate();

    // note: bl state is only set to ARMING at power-on/reset
    if (BL_ARMING == bl_opstate)
    {
      static const uint16_t ARMING_TIME_TOTAL = 0x0900u;
      static const uint16_t ARMING_TIME_DELAY = 0x0200u;
      static const uint16_t ARMING_TIME_MASK = 0x01C0u;
      static const uint16_t ARMING_BL_TIMING = 0x0010u;
      static uint16_t atimer = 0;

      BL_set_timing( ARMING_BL_TIMING ); // set to some small value (sampling vBatt measurement)

      if (atimer < ARMING_TIME_TOTAL)
      {
        atimer += 1;
        inp_dutycycle = 0;
        // brief delay after power-on
        if (atimer > ARMING_TIME_DELAY)
        {
          // hold the current/PWM at fixed level
          inp_dutycycle = PWM_PD_ARMING;
        }
        // turn off at regular interval to make distint beeping (more like clicking!) sound
        if (atimer & ARMING_TIME_MASK)
        {
          inp_dutycycle = 0;
        }
      }
      else
      {
        BL_reset(); // reset again to be sure motor-drive/PWM reinitialized
      }
    }
    else if (BL_STOPPED == BL_get_opstate())
    {
      // check if the motor startup should be initiated
      uint16_t bl_uispeed = BL_get_speed();
      if (bl_uispeed > 0)
      {
        BL_set_opstate( BL_ALIGN );
        BL_optimer = BL_TIME_ALIGN;

        // Set initial commutation timing period upon state transition.
        BL_set_timing( (uint16_t)BL_CT_RAMP_START );
      }
    }
    else if (BL_ALIGN == bl_opstate)
    {
      if (BL_optimer > 0)
      {
        inp_dutycycle = PWM_PD_ALIGN;
        BL_optimer -=1;
      }
      else
      {
        BL_set_opstate(BL_RAMPUP);
      }
    }
    else if (BL_RAMPUP == bl_opstate)
    {
      // grab the current commutation period setpoint to handoff to ramp control
      uint16_t bl_timing_setpt = BL_get_timing();

      // set target commutation timing period for end of ramp
      uint16_t tgt_timing_setpt = (uint16_t)BL_CT_RAMP_END;

      // only needs to ramp in 1 direction
      timing_ramp_control(bl_timing_setpt, tgt_timing_setpt);

      // Set PWM duty-cycle for rampup
      inp_dutycycle = PWM_PD_RAMPUP;

      if (bl_timing_setpt <= tgt_timing_setpt)
      {
        BL_set_opstate( BL_OPN_LOOP );
      }
    }

    else if (BL_OPN_LOOP == bl_opstate)
    {
      // get the present BL commutation timing setpoint
      uint16_t bl_timing_setpt = BL_get_timing();

      // control setpoint is Startup Speed, update the commutation timing
      // There is sort of an assumption here that the ramp-up over-shot the
      // speed (commutation period) and should now back off to the "startup"
      // timing. Unfortunately the exact values of those operating points are/were
      // tied to a static open-loop timing table for 1100kv motor at precisely 12.5v
      // Nonetheless syncing seems to work better to back the commutation timing
      //  off at this point. A new addition now is that here the real speed (PWM-DC) 
      // to ramped to the user input speed while waiting for sync to occur.
      timing_ramp_control(bl_timing_setpt, (uint16_t)BL_CT_STARTUP);

      // controller returns true upon successful control step
      if (FALSE != BL_cl_control(bl_timing_setpt))
      {
        BL_set_opstate( BL_CLS_LOOP );
        // start ramping speed (PWM duty-cycle) toward UI input speed
        inp_dutycycle = bl_get_ramped_speed(BL_get_speed());
      }
      else
      {
//        inp_dutycycle = PWM_get_dutycycle(); // hold the duty-cycle where it is
        // Needs to ramp the speed in order to converge with comm timing?
//        inp_dutycycle = bl_get_ramped_speed(BL_get_speed()); // this might be ok
        inp_dutycycle = bl_get_ramped_speed(PWM_PD_STARTUP); // shoot toward lower speed until CL kicks in?

      }
    }
    else if (BL_CLS_LOOP == bl_opstate)
    {
#define CL_FAULT_CNTR 2000u
      static const uint16_t FAULT_INCR = 20;
      static const uint16_t FAULT_DECR = 1;
      static uint16_t fault_counter = CL_FAULT_CNTR;

      // controller returns false upon failed control step
      if (FALSE != BL_cl_control(BL_get_timing()))
      {
        // if in-control, fill or reset leaky bucket
        if (fault_counter < CL_FAULT_CNTR)
        {
          // fault_counter += FAULT_INCR;
          fault_counter = CL_FAULT_CNTR;
        }
      }
      else
      {
        // tends to fault at cutover to CL ... use leaky-bucket to suppress
        if (fault_counter > 0)
        {
          fault_counter -= FAULT_DECR;
        }
        else
        {
          // ideally, the system might be able to remain running opeon-loop at current
          // setpoints even once control is lost
//          BL_set_opstate( BL_OPN_LOOP );
          Faultm_set(FAULT_1);
        }
      }
      // allow user speed input
      inp_dutycycle = bl_get_ramped_speed(BL_get_speed());
    }
    // end '0 == Faultm_get_status'
  }

  // pwm duty-cycle is propogated to timer peripheral at next commutation step
  PWM_set_dutycycle( inp_dutycycle );
}

/**
 * @brief  commutation sequence step (timer ISR callback)
 *
 * @details
 */
void BL_Commutation_Step(void)
{
  switch( BL_get_opstate() )
  {
  case BL_ARMING:
  case BL_ALIGN:
    // keep sector 0 on until timeout
    Sequence_Step_0();
    break;
  case BL_RAMPUP:
  case BL_OPN_LOOP:
  case BL_CLS_LOOP:
    Sequence_Step();
    break;

  case BL_STOPPED:
  case BL_NONE:
  default:
    break;
  }
}
/**@}*/ // defgroup

