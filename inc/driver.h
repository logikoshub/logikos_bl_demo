/**
  ******************************************************************************
  * @file driver.h
  * @brief Support functions for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2020
  ******************************************************************************
  *
  * BLAH BLAH BLAH
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef DRIVER_H
#define DRIVER_H

/* Includes ------------------------------------------------------------------*/

#include "pwm_stm8s.h"
#include "mdata.h"

/* defines -------------------------------------------------------------------*/
#define RX_BUFFER_SIZE  16  //how big should this be?

/* types --------------------------------------------------------------------*/

/* prototypes ---------------------------------------------------------------*/

void Driver_Step(void);
void Driver_Update(void);

uint16_t Driver_Get_ADC(void);

void Driver_on_PWM_edge(void);
void Driver_on_ADC_conv(void);

void Driver_on_capture_rise(void);
void Driver_on_capture_fall(void);

void Driver_set_pulse_dur(uint16_t duration);
uint16_t Driver_get_pulse_dur(void);

uint16_t Driver_get_pulse_perd(void);
uint16_t Driver_get_servo_position_counts(void);

void Driver_Get_Rx_It(void);
uint8_t Driver_Return_Rx_Buffer(void);
void Driver_Clear_Rx_Buffer_Element(uint8_t Location);

#endif // DRIVER_H
