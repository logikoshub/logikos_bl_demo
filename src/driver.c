/**
  ******************************************************************************
  * @file driver.c
  * @brief support functions for the BLDC motor control
  * @author Neidermeier
  * @version 
  * @date March-2020
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "parameter.h" // app defines

/* Private defines -----------------------------------------------------------*/

// see define of TIM2 PWM PD ... it set for 125uS @ clk 2Mhz
//#define PWM_TPRESCALER  TIM2_PRESCALER_1 //
// @ 8 Mhz
#define PWM_TPRESCALER  TIM2_PRESCALER_8 // 125 uS

#define PWM_DC_RAMPUP  52 // exp. determined

// nbr of steps required to commutate 3 phase
#define N_CSTEPS   6


// presently using T1 pd = 64uS
#define BLDC_OL_TM_LO_SPD   254  // start of ramp
// ten counts will speed up to about xxxx RPM (needs to be 2500 RPM or ~2.4mS

// WARNING MOTOR MAY LOCK UP!!!  ramp-up current MUST be higher
#define BLDC_OL_TM_HI_SPD    10  // looks like about 2600rpm (3.8 mS)

// manual adjustment of OL PWM DC ... limit (can get only to about 9 right now)
#define BLDC_OL_TM_MANUAL_HI_LIM   5     // WARNING ... she may blow capn'

// starting step-time for ramp-up 
#define RAMP_STEP_TIME0  0x1000


/* Public variables  ---------------------------------------------------------*/
u16 BLDC_OL_comm_tm;   // could be private

BLDC_STATE_T BLDC_State;


/* Private variables ---------------------------------------------------------*/
static uint16_t TIM2_pulse_0 ;
static uint16_t TIM2_pulse_1 ;
static uint16_t TIM2_pulse_2 ;

static u16 Ramp_Step_Tm; // reduced x2 each time but can't start any slower

static uint16_t global_uDC;
static u8 bldc_step = 0;

/* Private function prototypes -----------------------------------------------*/
void PWM_Config(void);
void bldc_move(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  */
void PWM_Set_DC(uint16_t pwm_dc)
{
    global_uDC = pwm_dc;

    if ( BLDC_RAMPUP == BLDC_State ){
// special sauce for rampup current  proably needs a current ramp too
        global_uDC = PWM_DC_RAMPUP;
    }
}

void PWM_set_outputs(u8 state0, u8 state1, u8 state2)
{
    TIM2_pulse_0 = state0 != 0 ? global_uDC : 0;
    TIM2_pulse_1 = state1 != 0 ? global_uDC : 0;
    TIM2_pulse_2 = state2 != 0 ? global_uDC : 0;

    PWM_Config();
}

/**
  * @brief  .
  * @par Parameters:
  * None
  * @retval void None
  *   GN: from UM0834 PWM example
  */
void PWM_Config(void)
{
    /* TIM2 Peripheral Configuration */
    TIM2_DeInit();

    /* Set TIM2 Frequency to 2Mhz ... and period to ?    ( @2Mhz, fMASTER period == @ 0.5uS) */
    TIM2_TimeBaseInit(PWM_TPRESCALER, ( TIM2_PWM_PD - 1 ) ); // PS==1, 499   ->  8khz (period == .000125)

    /* Channel 1 PWM configuration */
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_0, TIM2_OCPOLARITY_LOW );
    TIM2_OC1PreloadConfig(ENABLE);


    /* Channel 2 PWM configuration */
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_1, TIM2_OCPOLARITY_LOW );
    TIM2_OC2PreloadConfig(ENABLE);


    /* Channel 3 PWM configuration */
    TIM2_OC3Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, TIM2_pulse_2, TIM2_OCPOLARITY_LOW );
    TIM2_OC3PreloadConfig(ENABLE);

    /* Enables TIM2 peripheral Preload register on ARR */
    TIM2_ARRPreloadConfig(ENABLE);

    /* Enable TIM2 */
    TIM2_Cmd(ENABLE);

#if 1
// GN: tmp test
    TIM2->IER |= TIM2_IER_UIE; // Enable Update Interrupt (sets manually-counted
    // pwm at 20mS with DC related to commutation/test-pot))
#endif
}


/*
 *
 */
void BLDC_Spd_dec()
{
#ifdef BLDC_COMM_TEST 
    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm < BLDC_OL_TM_LO_SPD)
    {
        BLDC_OL_comm_tm += 1; // slower
    }
#endif 
}

/*
 *
 */
void BLDC_Spd_inc()
{
#ifdef BLDC_COMM_TEST 
    if (BLDC_ON == BLDC_State  && BLDC_OL_comm_tm >= BLDC_OL_TM_MANUAL_HI_LIM )
    {
        BLDC_OL_comm_tm -= 1; // faster
    }
#endif 
}


void BLDC_ramp_update(void)
{
    static const u16 RAMP_STEP_T1 = 0x0010; // step time at end of ramp
    static u16 ramp_step_tmr = 0;
    // on counter zero, decrement counter, start value divided by 2
    if ( 0 == ramp_step_tmr-- )
    {
        ramp_step_tmr = Ramp_Step_Tm;

        if (Ramp_Step_Tm > RAMP_STEP_T1)
        {
            Ramp_Step_Tm >>= 1;
        }
        //if (BLDC_OL_comm_tm > 0) // { // probably get by with an assert
        BLDC_OL_comm_tm -= 1;
        // }
    }
}

/*
 * BLDC Update: handle the BLDC state 
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
    static u16 count = 0;

    if ( ++count >= BLDC_OL_comm_tm )
    {
        // reset counter and step the BLDC state
        count = 0;
        BLDC_Step();
    }

    // state-machine: switch-case?
    if (BLDC_OFF == BLDC_State)
    {
        // reset commutation timer and ramp-up counters ready for ramp-up		
        BLDC_OL_comm_tm = BLDC_OL_TM_LO_SPD;
        Ramp_Step_Tm = RAMP_STEP_TIME0;
    }
    else
    if (BLDC_ON == BLDC_State)
    {
        // do ON stuff
    } // else
    else
    // ramp the speed if in rampup state
    if (BLDC_RAMPUP == BLDC_State)
    {
// TODO: the actual transition to ON state would be seeing the ramp-to speed 
// achieved in closed-loop operation 
        if (BLDC_OL_comm_tm > BLDC_OL_TM_HI_SPD) // state-transition trigger?
        {
            BLDC_ramp_update();
        }
        else
        {
            BLDC_State = BLDC_ON;
        }
    }
}

/*
 * drive /SD outputs and PWM channels
 */
void BLDC_Step(void)
{
    bldc_step += 1;
    bldc_step %= N_CSTEPS;

    if (global_uDC > 0)
    {
        bldc_move();
    }
    else // motor drive output has been disabled
    {
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(0, 0, 0);
    }
}

void bldc_move(void)
{
// /SD outputs on C5, C7, and G1
// wait until switch time arrives (watching for voltage on the floating line to cross 0)
    switch(bldc_step)
    {
    default:

    case 0:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(1, 0, 0);
        break;
    case 1:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(1, 0, 0);
        break;
    case 2:
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 1, 0);
        break;
    case 3:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR &=  ~(1<<1);
        PWM_set_outputs(0, 1, 0);
        break;
    case 4:
        GPIOC->ODR |=   (1<<5);
        GPIOC->ODR &=  ~(1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 0, 1);
        break;
    case 5:
        GPIOC->ODR &=  ~(1<<5);
        GPIOC->ODR |=   (1<<7);
        GPIOG->ODR |=   (1<<1);
        PWM_set_outputs(0, 0, 1);
        break;
    }
}