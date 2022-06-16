/**
  ******************************************************************************
  * @file sequence.h
  * @brief  Coordinates commutation switching sequence.
  * @author Neidermeier
  * @version
  * @date December-2020
  ******************************************************************************
  *
  * TBD
  *
  * <h2><center>&copy; COPYRIGHT 2112 asdf</center></h2>
  ******************************************************************************
  */
#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "system.h"


/* types -----------------------------------------------------------------*/

/* declarations ---------------------------------------------------------*/

/* macros ---------------------------------------------------------------*/

#define SEQ_N_CSTEPS    6


/* prototypes -----------------------------------------------------------*/

uint16_t Seq_Get_bemfR(void);
uint16_t Seq_Get_bemfF(void);

uint16_t Seq_Get_Vbatt(void);
int16_t Seq_get_timing_error(void);
bool Seq_get_timing_error_p(void);

void Sequence_Step(uint8_t step);

void Sequence_Step_0(void);
void Sequence_Step_1(void);
void Sequence_Step_2(void);
void Sequence_Step_3(void);
void Sequence_Step_4(void);
void Sequence_Step_5(void);

#endif // SEQUENCE_H
