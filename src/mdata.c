/**
  ******************************************************************************
  * @file mdata.c
  * @brief  Motor data table lookup
  * @author Neidermeier
  * @version
  * @date Nov-2020
  ******************************************************************************
  */
/**
 * \defgroup mdata Motor Model Data
  * @brief  Motor data table lookup
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "system.h" // dependency of motor data on cpu clock specific timer rate

/*
 * The open-loop commutation timing table is indexed by PWM duty cycle counts 
 * (i.e. [0:1:250). However, it is not expected that the motor could even run
 * open-loop into the higher operating speed, so the table only covers the lower
 * 50% of PWM duty-cycle range.
 * See "src/model.c" which is actully a Scilab script.
 * The function generates the data in Scilab and imported from csv:
 *
 *   y =  ( 4000 * EXP( -t/25 ) ) + 150
 *
 * Excel was used to find initial parameters using some data obtained 
 * experimentally by running the motor and manually syncing it using the
 * butttons and terminal inputs for ocmmutiation timing and PWM DC.
 *
 * sync range seems to be [32-68] (duty-cycle) with parameters roughly in these ranges:
 *  A     = [ 1800 : 2000 ]
 *  TAU   = [ 38 : 34 ]
 *  Y_INT = [ 25 : 30 ]
 *  X_INT = 0
 *
 *  (32-80)/(788-268) =  -0.0923077
 *
 * The exponential seems to do a better job at tracking over the needed
 * interval. Taking the coordinates at the start and end of the range over which
 * the motor is able to stay in sync would give the slope that could be tried for
 * a linear tracking function.
 *
 */
static const uint16_t OL_Timing[ /* TABLE_SIZE */ ] =
{
#if 1
1500, //   0, 0, 0, 5DC
1470, //   1, 1, 1, 5BE
1441, //   2, 1, 2, 5A1
1413, //   3, 2, 3, 585
1385, //   4, 2, 4, 569
1357, //   5, 3, 5, 54D
1330, //   6, 3, 6, 532
1304, //   7, 4, 7, 518
1278, //   8, 4, 8, 4FE
1253, //   9, 5, 9, 4E5
1228, //   10, 6, A, 4CC
1204, //   11, 6, B, 4B4
1180, //   12, 7, C, 49C
1157, //   13, 7, D, 485
1134, //   14, 8, E, 46E
1111, //   15, 8, F, 457
1089, //   16, 9, 10, 441
1068, //   17, 9, 11, 42C
1047, //   18, 10, 12, 417
1026, //   19, 11, 13, 402
1005, //   20, 11, 14, 3ED
986, //   21, 12, 15, 3DA
966, //   22, 12, 16, 3C6
947, //   23, 13, 17, 3B3
928, //   24, 13, 18, 3A0
910, //   25, 14, 19, 38E
892, //   26, 14, 1A, 37C
874, //   27, 15, 1B, 36A
857, //   28, 16, 1C, 359
840, //   29, 16, 1D, 348
823, //   30, 17, 1E, 337
807, //   31, 17, 1F, 327
791, //   32, 18, 20, 317
775, //   33, 18, 21, 307
760, //   34, 19, 22, 2F8
745, //   35, 19, 23, 2E9
730, //   36, 20, 24, 2DA
716, //   37, 21, 25, 2CC
701, //   38, 21, 26, 2BD
688, //   39, 22, 27, 2B0
674, //   40, 22, 28, 2A2
661, //   41, 23, 29, 295
648, //   42, 23, 2A, 288
635, //   43, 24, 2B, 27B
622, //   44, 24, 2C, 26E
610, //   45, 25, 2D, 262
598, //   46, 26, 2E, 256
586, //   47, 26, 2F, 24A
574, //   48, 27, 30, 23E
563, //   49, 27, 31, 233
552, //   50, 28, 32, 228
541, //   51, 28, 33, 21D
530, //   52, 29, 34, 212
520, //   53, 29, 35, 208
509, //   54, 30, 36, 1FD
499, //   55, 31, 37, 1F3
489, //   56, 31, 38, 1E9
480, //   57, 32, 39, 1E0
470, //   58, 32, 3A, 1D6
461, //   59, 33, 3B, 1CD
452, //   60, 33, 3C, 1C4
443, //   61, 34, 3D, 1BB
434, //   62, 34, 3E, 1B2
425, //   63, 35, 3F, 1A9
417, //   64, 36, 40, 1A1
409, //   65, 36, 41, 199
401, //   66, 37, 42, 191
393, //   67, 37, 43, 189
385, //   68, 38, 44, 181
377, //   69, 38, 45, 179
370, //   70, 39, 46, 172
363, //   71, 39, 47, 16B
355, //   72, 40, 48, 163
348, //   73, 41, 49, 15C
344, //   74, 41, 4A, 158
341, //   75, 42, 4B, 155
338, //   76, 42, 4C, 152
335, //   77, 43, 4D, 14F
332, //   78, 43, 4E, 14C
329, //   79, 44, 4F, 149
326, //   80, 44, 50, 146
323, //   81, 45, 51, 143
320, //   82, 46, 52, 140
317, //   83, 46, 53, 13D
314, //   84, 47, 54, 13A
311, //   85, 47, 55, 137
308, //   86, 48, 56, 134
305, //   87, 48, 57, 131
302, //   88, 49, 58, 12E
299, //   89, 49, 59, 12B
296, //   90, 50, 5A, 128
293, //   91, 51, 5B, 125
290, //   92, 51, 5C, 122
287, //   93, 52, 5D, 11F
284, //   94, 52, 5E, 11C
281, //   95, 53, 5F, 119
278, //   96, 53, 60, 116
275, //   97, 54, 61, 113
272, //   98, 54, 62, 110
269, //   99, 55, 63, 10D
266, //   100, 56, 64, 10A
263, //   101, 56, 65, 107
260, //   102, 57, 66, 104
257, //   103, 57, 67, 101
254, //   104, 58, 68, FE
251, //   105, 58, 69, FB
248, //   106, 59, 6A, F8
245, //   107, 59, 6B, F5
242, //   108, 60, 6C, F2
239, //   109, 61, 6D, EF
236, //   110, 61, 6E, EC
233, //   111, 62, 6F, E9
230, //   112, 62, 70, E6
227, //   113, 63, 71, E3
224, //   114, 63, 72, E0
221, //   115, 64, 73, DD
218, //   116, 64, 74, DA
215, //   117, 65, 75, D7
212, //   118, 66, 76, D4
209, //   119, 66, 77, D1
206, //   120, 67, 78, CE
203, //   121, 67, 79, CB
200, //   122, 68, 7A, C8
197, //   123, 68, 7B, C5
194, //   124, 69, 7C, C2
191, //   125, 69, 7D, BF
188, //   126, 70, 7E, BC
185, //   127, 71, 7F, B9
182, //   128, 71, 80, B6
#else
#include "model.h" // hack   headers are horrible
#endif
};

// move this into header file if needed publicly
#define OL_TIMING_TBL_SIZE    ( sizeof(OL_Timing) / sizeof(uint16_t) )

/**
 * @brief Table lookup for open-loop commutation timing
 *
 * @param index  Index into the table - motor speed i.e. PWM duty-cycle
 *
 * @return LUT value @ index
 * @retval -1 error
 */
uint16_t Get_OL_Timing(uint16_t index)
{
    // assert index < OL_TIMING_TBL_SIZE
    if ( index < OL_TIMING_TBL_SIZE )
    {
        return OL_Timing[ index ] * CTIME_SCALAR;
    }
    return (U16_MAX); // error
}

/**@}*/ // defgroup
