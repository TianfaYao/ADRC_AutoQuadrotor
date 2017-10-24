/*
 * File: fal.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Jun-2017 11:24:40
 */

/* Include files */
#include "rt_nonfinite.h"
#include "fal.h"
#include "fhan.h"

/* Function Declarations */
static float rt_powf_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_powf_snf(float u0, float u1)
{
  float y;
  float f0;
  float f1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f0 = (real32_T)fabs(u0);
    f1 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f0 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f0 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)pow(u0, u1);
    }
  }

  return y;
}

/*
 * Arguments    : float e
 *                float a
 *                float d
 * Return Type  : float
 */
float fal(float e, float a, float d)
{
  float f;
  float b_e;
  if ((real32_T)fabs(e) <= d) {
    f = e / rt_powf_snf(d, 1.0F - a);
  } else {
    if (e < 0.0F) {
      b_e = -1.0F;
    } else if (e > 0.0F) {
      b_e = 1.0F;
    } else if (e == 0.0F) {
      b_e = 0.0F;
    } else {
      b_e = e;
    }

    f = b_e * rt_powf_snf((real32_T)fabs(e), a);
  }

  return f;
}

/*
 * File trailer for fal.c
 *
 * [EOF]
 */
