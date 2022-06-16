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

// Battery volts measurement out-of-range threshold - half of 10-bit ADC range?
#define BL_VSYS_OOR_THRSH  0x0200

/*
 * Percent PWM must be converted to PWM Percent-duty-cycle expressed in counts.
 */
#define PWM_PCNT_ARMING     5.0 // experimented effect on voltage measurement
#define PWM_PCNT_ALIGN     25.0
#define PWM_PCNT_RAMPUP    14.0
#define PWM_PCNT_STARTUP   12.0
#define PWM_PCNT_SHUTOFF    3.0 // stalls at ~3.3%

// PWM period times for various operating thresholds
#define PWM_PD_ARMING    PWM_GET_PULSE_COUNTS( PWM_PCNT_ARMING )
#define PWM_PD_ALIGN     PWM_GET_PULSE_COUNTS( PWM_PCNT_ALIGN )
#define PWM_PD_RAMPUP    PWM_GET_PULSE_COUNTS( PWM_PCNT_RAMPUP )
#define PWM_PD_STARTUP   PWM_GET_PULSE_COUNTS( PWM_PCNT_STARTUP )
#define PWM_PD_SHUTOFF   PWM_GET_PULSE_COUNTS( PWM_PCNT_SHUTOFF )


/* types ---------------------------------------------------------------------*/

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
BL_state_t;

/**
 * @brief Accessor for state variable.
 *
 * @return state value
 */
typedef struct
{
  BL_state_t bL_opstate;
  uint16_t bl_sys_voltage;
  uint16_t bl_motor_speed;
  uint16_t bl_comm_period;
}
BL_status_t;


/* prototypes ----------------------------------------------------------------*/

/**
  * @brief Accessor for commutation period
  *
  * @return commutation period
  */
uint16_t BL_get_timing(void);
void BL_set_timing( uint16_t u16);

void BL_set_opstate(uint8_t opstate);
uint8_t BL_get_opstate(void);

void BL_timing_step_slower(void);
void BL_timing_step_faster(void);

void BL_set_speed(uint16_t ui_mspeed_counts);
uint16_t BL_get_speed(void);

void BL_reset(void);

BL_status_t *BL_get_status(void);

uint8_t BL_get_ct_mode(void);

/**
 * @brief Fixed-rate controller update (timer ISR callback)
 */
void BL_state_control(void);

/**
 * @brief Commutation sequence step (timer ISR callback)
 */
void BL_commutation_step(void);

#endif // BLDC_H
