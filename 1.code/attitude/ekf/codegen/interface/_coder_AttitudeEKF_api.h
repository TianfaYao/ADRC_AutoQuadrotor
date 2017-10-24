/* 
 * File: _coder_AttitudeEKF_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 23-Apr-2017 01:32:45 
 */

#ifndef ___CODER_ATTITUDEEKF_API_H__
#define ___CODER_ATTITUDEEKF_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void AttitudeEKF_initialize(emlrtContext *aContext);
extern void AttitudeEKF_terminate(void);
extern void AttitudeEKF_atexit(void);
extern void AttitudeEKF_api(const mxArray *prhs[13], const mxArray *plhs[5]);
extern void AttitudeEKF(unsigned char approx_prediction, unsigned char use_inertia_matrix, unsigned char zFlag[3], float dt, float z[9], float q_rotSpeed, float q_rotAcc, float q_acc, float q_mag, float r_gyro, float r_accel, float r_mag, float J[9], float xa_apo[12], float Pa_apo[144], float Rot_matrix[9], float eulerAngles[3], float debugOutput[4]);
extern void AttitudeEKF_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_AttitudeEKF_api.h 
 *  
 * [EOF] 
 */
