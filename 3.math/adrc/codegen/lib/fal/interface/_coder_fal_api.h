/* 
 * File: _coder_fal_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 28-Jun-2017 11:24:40 
 */

#ifndef ___CODER_FAL_API_H__
#define ___CODER_FAL_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void fal_initialize(emlrtContext *aContext);
extern void fal_terminate(void);
extern void fal_atexit(void);
extern void fal_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
extern float fal(float e, float a, float d);
extern void fhan_api(const mxArray * const prhs[4], const mxArray *plhs[1]);
extern float fhan(float x1, float x2, float r, float h);
extern void fal_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_fal_api.h 
 *  
 * [EOF] 
 */
