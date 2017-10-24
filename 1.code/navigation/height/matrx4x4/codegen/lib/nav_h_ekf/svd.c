/*
 * File: svd.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 10:29:44
 */

/* Include files */
#include "rt_nonfinite.h"
#include "nav_h_ekf.h"
#include "svd.h"

/* Function Declarations */
static void c_eml_xscal(float a, float *x, int ix0);

/* Function Definitions */

/*
 * Arguments    : float a
 *                float *x
 *                int ix0
 * Return Type  : void
 */
static void c_eml_xscal(float a, float *x, int ix0)
{
  int k;
  k = ix0;
  while (k <= ix0) {
    *x *= a;
    k = 2;
  }
}

/*
 * Arguments    : float x
 *                float y
 * Return Type  : float
 */
float b_eml_div(float x, float y)
{
  return x / y;
}

/*
 * Arguments    : float A
 *                float *U
 *                float *S
 *                float *V
 * Return Type  : void
 */
void svd(float A, float *U, float *S, float *V)
{
  int m;
  int iter;
  m = 1;
  *S = A;
  *U = 1.0F;
  *V = 1.0F;
  if (A != 0.0F) {
    *S = (real32_T)fabs(A);
    *U = b_eml_div(A, *S);
  }

  iter = 0;
  while ((m > 0) && (!(iter >= 75))) {
    if (*S < 0.0F) {
      *S = -*S;
      c_eml_xscal(-1.0F, V, 1);
    }

    iter = 0;
    m = 0;
  }
}

/*
 * File trailer for svd.c
 *
 * [EOF]
 */
