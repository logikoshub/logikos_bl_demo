/**
  ******************************************************************************
  * @file driver.h
  * @brief  
  * @author
  * @version 
  * @date    
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

#include "stm8s.h"


/* Private defines -----------------------------------------------------------*/

// divider: 33k/18k
//  18/(18+33)=0.35
// 0.35 * 14.1v = 4.98
// 4.98 / 2 = 2.48v ........... 1/2 Vdc in proportion to the resister divider
//  2.48v/5v =  x counts / 1024 ocunts so 1/2 Vdc is equivalent to x counts ...
//   x = 1024 * 2.48/5 = 509   (0x01FD)
#define DC_HALF_REF         0x01FD



/* Private types -----------------------------------------------------------*/

/*
 * defines
 */


/*
 * types
 */


/*
 * variables
 */


/*
 * prototypes
 */

void Driver_Step(void);
void Driver_Update(void);
uint16_t Driver_Get_ADC(void);
uint16_t Driver_Get_Vbatt(void);

void bemf_samp_start( void );
void bemf_samp_get(void);


#endif // DRIVER_H
