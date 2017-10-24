/*
 * File: nav_h_ekf_initialize.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 10:29:44
 */

/* Include files */
#include "rt_nonfinite.h"
#include "nav_h_ekf.h"
#include "nav_h_ekf_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_h_ekf_initialize(void)
{
  rt_InitInfAndNaN(8U);
  nav_h_ekf_init();
}

/*
 * File trailer for nav_h_ekf_initialize.c
 *
 * [EOF]
 */
