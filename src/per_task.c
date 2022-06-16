/**
  ******************************************************************************
  * @file per_task.c
  * @brief Background task / periodic task
  * @author Neidermeier
  * @version
  * @date Dec-2020
  ******************************************************************************
  */
/**
 * \defgroup per_task Periodic Task
 * @brief Background task / periodic task
 * @{
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stddef.h> // NULL
// app headers - there are several needed for logging system data
#include "mcu_stm8s.h"
#include "sequence.h"
#include "bldc_sm.h"
#include "faultm.h"
#include "driver.h"
#include "spi_stm8s.h"
#include "pdu_manager.h"

/* Private defines -----------------------------------------------------------*/
// Stall-voltage threshold must be set low enuogh to avoid false-positive as
// the voltage droops on startup and transition out of ramp.
// The fault can be tested by letting the spinning prop strike a business card.
// Example of typical measure Vsys with given voltage divider:
//
//  Vbatt == 12.5v 10k/(33k+10k) * 12.5v = 2.91v
//  2.9v * 1024 / 3.3v = $0384
//  observed stall voltage ~$02F0
#define V_SHUTDOWN_THR      0x0280 // GN: 6/14/2022

// each +/- press of speed keys inc/decrements this amount (100% -> 1000/4 = 250 steps)
#define MSPEED_PCNT_INCREM_STEP    (PWM_PERIOD_COUNTS / 250)


/* Private function prototypes -----------------------------------------------*/
// forward declarations for UI input handers
static void auto_mode(void);
static void manual_mode(void);
static void timing_plus(void);
static void timing_minus(void);
static void spd_plus(void);
static void spd_minus(void);
static void m_stop(void);
static void m_start(void);
static void help_me(void);


/* Private types     ---------------------------------------------------------*/
/**
 * @brief Data type for the key handler function.
 */
typedef void (*ui_handlrp_t)( void );

/**
 * @brief Data type for the key code lookup table.
 */
typedef enum
{
  MANUAL_MODE = 'm',
  AUTO_MODE   = 'a',
  COMM_PLUS   = ']',
  COMM_MINUS  = '[',
  M_STOP      = ' ', // space bar
  M_START     = '/', // /
  SPD_PLUS    = '.', // >
  SPD_MINUS   = ',', // <
  HELP_ME     = '?',
  K_UNDEFINED = -1
}
ui_keycode_t;

/**
 * @brief Data type for the key handler table.
 */
typedef struct
{
  ui_keycode_t   key_code;  /**< Key code. */
  ui_handlrp_t   phandler;  /**< Pointer to handler function. */
}
ui_key_handler_t;

/* Private variables ---------------------------------------------------------*/

// shared between ISR and non-ISR context
static volatile uint8_t TaskRdy; // flag for timer interrupt for BG task timing

static BL_status_t bl_status;

static uint8_t Log_Level;
static uint16_t UI_Speed; // motor percent speed input from servo or remote UI

#define KEYBOARD_DETECT_WINDOW 60 // 60 * 0.167 mS = 1 second
static uint8_t Radio_detect_timer;
static bool Enable_radio_input;

/**
 * @brief Lookup table for UI input handlers
 */
static const ui_key_handler_t ui_keyhandlers_tb[] =
{
  {AUTO_MODE,   auto_mode},
  {MANUAL_MODE, manual_mode},
  {COMM_PLUS,   timing_plus},
  {COMM_MINUS,  timing_minus},
  {SPD_PLUS,    spd_plus},
  {SPD_MINUS,   spd_minus},
  {M_STOP,      m_stop},
  {M_START,     m_start},
  {HELP_ME,     help_me}
};

// macros to help make the LUT slightly more encapsulated
#define _SIZE_K_LUT  ( sizeof( ui_keyhandlers_tb ) / sizeof( ui_key_handler_t ) )
#define _GET_KEY_CODE( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].key_code
#define _GET_UI_HDLRP( _INDEX_ )  ui_keyhandlers_tb[ _INDEX_ ].phandler


/* Private functions ---------------------------------------------------------*/
/**
 * @brief Print one line to the debug serial port.
 * @note: NOT appropriate in either an ISR or critical section because of printf
 *  to serial terminal is blocking.
 *
 * @param zeroflag set 1 to zero the line count
 */
static void Log_println(int zrof)
{
  static uint16_t Line_Count = 0;
  // if flag is set then reset line counter
  if ( 0 != zrof)
  {
    Line_Count = 0;
  }
  // if logger is enabled (level>0) then invoke its output
  if ( Log_Level > 0)
  {
    printf(
      "{%04X) PWMDC%=%X CtmCt=%04X BLdc=%04X Vs=%04X Sflt=%X RCsigCt=%04X MspdCt=%04u ERR=%04X ST=%u BR=%04X BF=%04X \r\n",
      Line_Count++,  // increment line count
      PWM_get_dutycycle(),
      BL_get_timing(),
      BL_get_speed(),
      bl_status.bl_sys_voltage,
      (int)Faultm_get_status(),

      Driver_get_pulse_dur(),
      Driver_get_servo_position_counts(), // servo posn counts -> PWM pulse DC counts [0:1023]

      Seq_get_timing_error(),
      (uint16_t)BL_get_opstate(),
      Seq_Get_bemfR(),
      Seq_Get_bemfF()
    );
    Log_Level -= 1;
  }
}

/*
 * Handlers for UI events
 * Must be short as they are invoked in ISR context
 */
/*
 * select manual control mode
*/
static void manual_mode(void)
{
  // disconnects the controller, leaving all operating points where they are
  BL_set_opstate(BL_MANUAL);
}
/*
 * select auto control mode
*/
static void auto_mode(void)
{
  // sets open loop, from where the controller will transition to closed-loop
  // if motor will sync
  BL_set_opstate(BL_OPN_LOOP);
}

/*
 * increase commutation period (manual control)
 */
static void timing_plus(void)
{
  BL_timing_step_slower();
}
/*
 * decrease commutation period (manual control)
 */
static void timing_minus(void)
{
  BL_timing_step_faster();
}

/*
 * motor start
 */
static void m_start(void)
{
  UI_Speed = (uint16_t)(PWM_PD_STARTUP + MSPEED_PCNT_INCREM_STEP);
  BL_set_speed( UI_Speed );
}
/*
 * motor stop
 */
static void m_stop(void)
{
  printf("\r\nStopped!\r\n");
  BL_reset();

  UI_Speed = 0;

  Log_Level = 1; // allow one more status line printfd to terminal then stops log output
  Log_println(1 /* clear line count */ );
}

/*
 * motor speed increment
 */
static void spd_plus(void)
{
  uint16_t speed = UI_Speed + (uint16_t)MSPEED_PCNT_INCREM_STEP;
  // protect against overflow
  if (speed > UI_Speed)
  {
    UI_Speed = speed;
  }
  Log_Level = 1;

}
/*
 * motor speed decrement (manual control)
 */
static void spd_minus(void)
{
  uint16_t speed = UI_Speed - (uint16_t)MSPEED_PCNT_INCREM_STEP;
  // protect against underflow
  if (speed < UI_Speed)
  {
    UI_Speed = speed;
  }
  Log_Level = 1;
}

/*
 * handle terminal input - these are simple 1-key inputs for now
 */
static ui_handlrp_t handle_term_inp(void)
{
  ui_handlrp_t fp = NULL;
  char key;

// Uses non-blocking/non-buffered scan for key input similar to getch()
  if (SerialKeyPressed(&key))
  {
    int n;
    for (n = 0; n < _SIZE_K_LUT ; n++)
    {
      if (key == (char)_GET_KEY_CODE( n ))
      {
// any terminal output specific to the key or handler needs to be done here and
// not in the handler itself because the handler is to be called from w/i the CS
        fp =_GET_UI_HDLRP( n );
        break;
      }
    }
// anykey ...
    Log_Level = 255;// by default, any-key enables continous/verbose log
    if (Radio_detect_timer < KEYBOARD_DETECT_WINDOW)
    {
      Enable_radio_input = FALSE;
      Radio_detect_timer = KEYBOARD_DETECT_WINDOW;
    }
  }
  return fp;
}

void help_me(void)
{
  printf("\r\n");
  printf("----------------------------------------------\r\n");
  printf("BL Motor Control Version %d\r\n", BL_SW_VERSION);
  printf("  Detected Vbatt 0x%04X\r\n", bl_status.bl_sys_voltage);
  printf("  Keys:\r\n");
  printf("     / (slash):  start\r\n");
  printf("     <    >   :  speed-/speed+\r\n");
  printf("     m        :  toggle auto/manual control\r\n");
  printf("     [    ]   :  speed+/speed- (manual commutation control)\r\n");
  printf("     Space Bar:  stop\r\n");
  printf("----------------------------------------------\r\n");
  printf("\r\n");
}

/**
 * @brief Print the software information to the terminal
 * @note needs to report s/w version
 */
void Print_banner(void)
{
  help_me();
}

/**
 * @brief  Extern function for system reset
 */
void UI_Stop(void)
{
  m_stop();
  BL_set_opstate( BL_ARMING );  // set the initial control-state
  Log_Level = 10; // show some information on the terminal
}

/**
 * @brief  The User Interface task
 *
 * @details   Service the UI and communication handlers. Invoked in the
 *   execution context of 'main()' (background task)
 */
static void Periodic_task(void)
{
  static bool rf_enabled = FALSE;
  static uint16_t servo_pulse_sma = 0;

// Invoke the terminal input and ui speed subroutines.
// If there is a valid key input, a function pointer to the input handler is
// returned. This is done prior to entering a Critical Section (DI/EI) in which
// it will then be safe to invoke the input handler function (e.g. can call
// subfunctions that may be messing with global variables e.g. motor speed etc.
  ui_handlrp_t fp = handle_term_inp();

  disableInterrupts();  //////////////// DI

  bl_status = BL_get_status();

  if (NULL != fp)
  {
    fp();
  }

  // passes the UI percent motor speed to the BL controller
  if (Radio_detect_timer < KEYBOARD_DETECT_WINDOW)
  {
    // if any key input inside keyboard detect window, Enable Manual Speed will set True
    Radio_detect_timer += 1;
    // todo: if radio detected ... stop looking for key input
    if ( Driver_get_pulse_dur() > TCC_TIME_DETECT )
    {
      Radio_detect_timer = KEYBOARD_DETECT_WINDOW;
      Enable_radio_input = TRUE;
    }
  }
  else
  {
    uint16_t cmd_speed;
    if (FALSE != Enable_radio_input)
    {
      servo_pulse_sma = (Driver_get_servo_position_counts() + servo_pulse_sma) / 2;
      cmd_speed = servo_pulse_sma;
    }
    else
    {
      cmd_speed = UI_Speed;
    }
    BL_set_speed(cmd_speed);
  }

  enableInterrupts();  ///////////////// EI EI O

#if defined( UNDERVOLTAGE_FAULT_ENABLED )
  // update system voltage diagnostic - check plausibilty of Vsys
  if (bl_status.bl_sys_voltage > BL_VSYS_OOR_THRSH)
  {
    Faultm_upd(
      VOLTAGE_NG, (faultm_assert_t)(bl_status.bl_sys_voltage < V_SHUTDOWN_THR));
  }
#endif
}

/**
 * @brief  Run Periodic Task if ready
 *
 * @details
 * Called in non-ISR context - checks the background task ready flag which if !0
 * will invoke the Periodic Task function.
 * @note  Called at ~60 Hz (0.0167 ms) - see Driver_Update()
 * @return  True if task ran (allows caller to also sync w/ the time period)
 */
uint8_t Task_Ready(void)
{
  static uint8_t framecount = 0;
  static bool is_first = TRUE;

#ifdef UART_IT_RXNE_ENABLE
  Pdu_Manager_Handle_Rx();
#endif

  if (is_first)
  {
    is_first = FALSE;
    Print_banner();
    Enable_radio_input = FALSE;
  }

  if (0 != TaskRdy)
  {
    TaskRdy = FALSE;
    Periodic_task();

    framecount += 1;

    // periodic task @ ~60 Hz - modulus 0x10 -> 16 * 0.016 s = 0.267 seconds (~4 Hz)
    if (0 == (framecount % 0x10))
    {
      /* Toggles LED to verify task timing */
      //GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PIN);

      Log_println(0); // note: no printf to serial terminal inside a CS
    }
    else if (4 == (framecount % 0x80)) // don't let it fall on the modulus of Log Print
    {
      // SPI can TX more frequently than Log Print but don't let both on the same frames
#if SPI_ENABLED == SPI_STM8_MASTER
      SPI_controld();
#endif
    }
    return TRUE;
  }
  return FALSE;
}

/**
 * @brief  Trigger background task.
 *
 * @details
 * Called in ISR context - sets the background task ready flag which when seen
 * by polling Task_Ready in background task will invoke the Periodic Task function.
 */
void Periodic_Task_Wake(void)
{
  TaskRdy = TRUE; // notify background process
}
/**@}*/ // defgroup
