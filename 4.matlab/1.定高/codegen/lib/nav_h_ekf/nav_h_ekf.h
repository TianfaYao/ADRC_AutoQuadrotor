/*
 * File: nav_h_ekf.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 02:29:18
 */

#ifndef __NAV_H_EKF_H__
#define __NAV_H_EKF_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "nav_h_ekf_types.h"

/* Function Declarations */
extern void nav_h_ekf(const float Q[16], const float H[8], const float R[4],
                      const float F[16], const float I[16], const float Z[2],
                      float *h, float *v, float *a);
extern void nav_h_ekf_init(void);

#endif

/*
 * File trailer for nav_h_ekf.h
 *
 * [EOF]
 */
