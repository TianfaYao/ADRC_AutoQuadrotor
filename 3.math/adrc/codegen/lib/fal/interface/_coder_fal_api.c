/*
 * File: _coder_fal_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Jun-2017 11:24:40
 */

/* Include files */
#include "_coder_fal_api.h"

/* Function Declarations */
static float emlrt_marshallIn(const emlrtStack *sp, const mxArray *e, const char
  *identifier);
static float b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *emlrt_marshallOut(const float u);
static float c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void fal_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void fal_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void fal_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  fal_xil_terminate();
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void fal_api(const mxArray * const prhs[3], const mxArray *plhs[1])
{
  float e;
  float a;
  float d;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  e = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "e");
  a = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "a");
  d = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "d");

  /* Invoke the target function */
  e = fal(e, a, d);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(e);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *e
 *                const char *identifier
 * Return Type  : float
 */
static float emlrt_marshallIn(const emlrtStack *sp, const mxArray *e, const char
  *identifier)
{
  float y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(e), &thisId);
  emlrtDestroyArray(&e);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float
 */
static float b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  float y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const float u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
  *(float *)mxGetData(m0) = u;
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[4]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void fhan_api(const mxArray * const prhs[4], const mxArray *plhs[1])
{
  float x1;
  float x2;
  float r;
  float h;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  x1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "x1");
  x2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "x2");
  r = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "r");
  h = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "h");

  /* Invoke the target function */
  x1 = fhan(x1, x2, r, h);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(x1);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float
 */
static float c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  float ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 0U, 0);
  ret = *(float *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_fal_api.c
 *
 * [EOF]
 */
