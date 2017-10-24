/*
 * File: nav_h_ekf.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 11:01:12
 */

#ifndef __NAV_H_EKF_H__
#define __NAV_H_EKF_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "nav_h_ekf_types.h"

/* Function Declarations */
extern void nav_h_ekf(const float Q[9], const float H[6], const float R[4],
                      const float F[9], const float I[9], const float Z[2],
                      unsigned char up, float *h, float *v, float *a);
extern void nav_h_ekf_init(void);

#endif

/*
 * File trailer for nav_h_ekf.h
 *
 * [EOF]
 */
