/* 
 * File: _coder_nav_h_ekf_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 03-Jul-2017 10:29:44 
 */

#ifndef ___CODER_NAV_H_EKF_API_H__
#define ___CODER_NAV_H_EKF_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void nav_h_ekf_initialize(emlrtContext *aContext);
extern void nav_h_ekf_terminate(void);
extern void nav_h_ekf_atexit(void);
extern void nav_h_ekf_api(const mxArray *prhs[7], const mxArray *plhs[4]);
extern void nav_h_ekf(float Q[16], float H[8], float R[4], float F[16], float I[16], float Z[2], unsigned char up, float *h, float *v, float *a, float *a_a);
extern void nav_h_ekf_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_nav_h_ekf_api.h 
 *  
 * [EOF] 
 */
