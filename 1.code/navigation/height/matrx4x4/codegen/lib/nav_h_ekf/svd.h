/*
 * File: svd.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 10:29:44
 */

#ifndef __SVD_H__
#define __SVD_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "nav_h_ekf_types.h"

/* Function Declarations */
extern float b_eml_div(float x, float y);
extern void svd(float A, float *U, float *S, float *V);

#endif

/*
 * File trailer for svd.h
 *
 * [EOF]
 */
