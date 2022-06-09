/**
  ******************************************************************************
  * @file bldc_sm.h
  * @brief state-manager for motor control
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
#ifndef BLDC_H
#define BLDC_H

/* Includes ------------------------------------------------------------------*/
#include "system.h"

#ifdef UNIT_TEST
#include <stdint.h> // was supposed to go thru sytsem.h :(
#endif

/* macros --------------------------------------------------------------------*/
#define PWM_BL_STOP  U16_MAX


/*
 * Percent PWM must be converted to PWM Percent-duty-cycle expressed in counts.
 */
#define PWM_PCNT_ARMING     8.5 // TBD
#define PWM_PCNT_ALIGN     25.0
#define PWM_PCNT_RAMPUP    14.0
#define PWM_PCNT_STARTUP   12.0
#define PWM_PCNT_SHUTOFF    9.0 // stalls at ~8%

// define PWM pulse times for operation states
#define PWM_PD_ARMING    PWM_GET_PULSE_COUNTS( PWM_PCNT_ARMING )
#define PWM_PD_ALIGN     PWM_GET_PULSE_COUNTS( PWM_PCNT_ALIGN )
#define PWM_PD_RAMPUP    PWM_GET_PULSE_COUNTS( PWM_PCNT_RAMPUP )
#define PWM_PD_STARTUP   PWM_GET_PULSE_COUNTS( PWM_PCNT_STARTUP )
#define PWM_PD_SHUTOFF   PWM_GET_PULSE_COUNTS( PWM_PCNT_SHUTOFF )


/* types ---------------------------------------------------------------------*/


/* prototypes ----------------------------------------------------------------*/

/**
 * @brief Accessor for state variable.
 *
 * @return state value
 */
typedef enum
{
  BL_NOT_RUNNING,
  BL_IS_RUNNING
}
BL_RUNSTATE_t;

/**
 * @brief Type for BL operating state.
 */
typedef enum
{
  BL_NONE,
  BL_ARMING,
  BL_STOPPED,
  BL_ALIGN,
  BL_RAMPUP,
  BL_OPN_LOOP,
  BL_CLS_LOOP,
  BL_MANUAL,
  BL_INVALID
}
BL_State_T;

/**
  * @brief Accessor for commutation period.
  *
  * @return commutation period
  */
uint16_t BL_get_timing(void);
void BL_set_timing( uint16_t  u16);

void BL_set_opstate(uint8_t opstate);
uint8_t BL_get_opstate(void);

void BL_timing_step_slower(void);
void BL_timing_step_faster(void);

void BL_set_speed(uint16_t ui_mspeed_counts);
uint16_t BL_get_speed(void);

void BL_reset(void);

BL_RUNSTATE_t BL_get_state(void);
uint8_t BL_get_ct_mode(void);

/**
 * @brief fixed-rate controller update (timer ISR callback).
 */
void BL_State_Ctrl(void);

/**
 * @brief commutation sequence step (timer ISR callback)
 */
void BL_Commutation_Step(void);


#endif // BLDC_H
