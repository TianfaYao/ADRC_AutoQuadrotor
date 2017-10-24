/*
 * File: fal_initialize.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Jun-2017 11:24:40
 */

/* Include files */
#include "rt_nonfinite.h"
#include "fal.h"
#include "fhan.h"
#include "fal_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void fal_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * File trailer for fal_initialize.c
 *
 * [EOF]
 */
