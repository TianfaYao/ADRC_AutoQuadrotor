/*
 * File: _coder_AttitudeEKF_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 23-Apr-2017 01:32:45
 */

/* Include files */
#include "_coder_AttitudeEKF_api.h"

/* Function Declarations */
static unsigned char emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *approx_prediction, const char *identifier);
static unsigned char b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static unsigned char (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *zFlag, const char *identifier))[3];
static unsigned char (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3];
static float e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier);
static float f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static float (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z, const
  char *identifier))[9];
static float (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static float (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *J, const
  char *identifier))[9];
static float (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[9];
static const mxArray *emlrt_marshallOut(const float u[12]);
static const mxArray *b_emlrt_marshallOut(const float u[144]);
static const mxArray *c_emlrt_marshallOut(const float u[9]);
static const mxArray *d_emlrt_marshallOut(const float u[3]);
static const mxArray *e_emlrt_marshallOut(const float u[4]);
static unsigned char k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static unsigned char (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId))[3];
static float m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static float (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];
static float (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void AttitudeEKF_initialize(emlrtContext *aContext)
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
void AttitudeEKF_terminate(void)
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
void AttitudeEKF_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  AttitudeEKF_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[13]
 *                const mxArray *plhs[5]
 * Return Type  : void
 */
void AttitudeEKF_api(const mxArray *prhs[13], const mxArray *plhs[5])
{
  float (*xa_apo)[12];
  float (*Pa_apo)[144];
  float (*Rot_matrix)[9];
  float (*eulerAngles)[3];
  float (*debugOutput)[4];
  unsigned char approx_prediction;
  unsigned char use_inertia_matrix;
  unsigned char (*zFlag)[3];
  float dt;
  float (*z)[9];
  float q_rotSpeed;
  float q_rotAcc;
  float q_acc;
  float q_mag;
  float r_gyro;
  float r_accel;
  float r_mag;
  float (*J)[9];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  xa_apo = (float (*)[12])mxMalloc(sizeof(float [12]));
  Pa_apo = (float (*)[144])mxMalloc(sizeof(float [144]));
  Rot_matrix = (float (*)[9])mxMalloc(sizeof(float [9]));
  eulerAngles = (float (*)[3])mxMalloc(sizeof(float [3]));
  debugOutput = (float (*)[4])mxMalloc(sizeof(float [4]));
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, false, -1);
  prhs[12] = emlrtProtectR2012b(prhs[12], 12, false, -1);

  /* Marshall function inputs */
  approx_prediction = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]),
    "approx_prediction");
  use_inertia_matrix = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]),
    "use_inertia_matrix");
  zFlag = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "zFlag");
  dt = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "dt");
  z = g_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "z");
  q_rotSpeed = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "q_rotSpeed");
  q_rotAcc = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "q_rotAcc");
  q_acc = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "q_acc");
  q_mag = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "q_mag");
  r_gyro = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "r_gyro");
  r_accel = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "r_accel");
  r_mag = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "r_mag");
  J = i_emlrt_marshallIn(&st, emlrtAlias(prhs[12]), "J");

  /* Invoke the target function */
  AttitudeEKF(approx_prediction, use_inertia_matrix, *zFlag, dt, *z, q_rotSpeed,
              q_rotAcc, q_acc, q_mag, r_gyro, r_accel, r_mag, *J, *xa_apo,
              *Pa_apo, *Rot_matrix, *eulerAngles, *debugOutput);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*xa_apo);
  plhs[1] = b_emlrt_marshallOut(*Pa_apo);
  plhs[2] = c_emlrt_marshallOut(*Rot_matrix);
  plhs[3] = d_emlrt_marshallOut(*eulerAngles);
  plhs[4] = e_emlrt_marshallOut(*debugOutput);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *approx_prediction
 *                const char *identifier
 * Return Type  : unsigned char
 */
static unsigned char emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *approx_prediction, const char *identifier)
{
  unsigned char y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(approx_prediction), &thisId);
  emlrtDestroyArray(&approx_prediction);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : unsigned char
 */
static unsigned char b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  unsigned char y;
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *zFlag
 *                const char *identifier
 * Return Type  : unsigned char (*)[3]
 */
static unsigned char (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *zFlag, const char *identifier))[3]
{
  unsigned char (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(zFlag), &thisId);
  emlrtDestroyArray(&zFlag);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : unsigned char (*)[3]
 */
  static unsigned char (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *
  u, const emlrtMsgIdentifier *parentId))[3]
{
  unsigned char (*y)[3];
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dt
 *                const char *identifier
 * Return Type  : float
 */
static float e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char *identifier)
{
  float y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float
 */
static float f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  float y;
  y = m_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *z
 *                const char *identifier
 * Return Type  : float (*)[9]
 */
static float (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *z, const
  char *identifier))[9]
{
  float (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(z), &thisId);
  emlrtDestroyArray(&z);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[9]
 */
  static float (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  float (*y)[9];
  y = n_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *J
 *                const char *identifier
 * Return Type  : float (*)[9]
 */
static float (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *J, const
  char *identifier))[9]
{
  float (*y)[9];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = j_emlrt_marshallIn(sp, emlrtAlias(J), &thisId);
  emlrtDestroyArray(&J);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[9]
 */
  static float (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[9]
{
  float (*y)[9];
  y = o_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u[12]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const float u[12])
{
  const mxArray *y;
  static const int iv0[1] = { 0 };

  const mxArray *m0;
  static const int iv1[1] = { 12 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const float u[144]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const float u[144])
{
  const mxArray *y;
  static const int iv2[2] = { 0, 0 };

  const mxArray *m1;
  static const int iv3[2] = { 12, 12 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const float u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const float u[9])
{
  const mxArray *y;
  static const int iv4[2] = { 0, 0 };

  const mxArray *m2;
  static const int iv5[2] = { 3, 3 };

  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv4, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv5, 2);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const float u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const float u[3])
{
  const mxArray *y;
  static const int iv6[1] = { 0 };

  const mxArray *m3;
  static const int iv7[1] = { 3 };

  y = NULL;
  m3 = emlrtCreateNumericArray(1, iv6, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)u);
  emlrtSetDimensions((mxArray *)m3, iv7, 1);
  emlrtAssign(&y, m3);
  return y;
}

/*
 * Arguments    : const float u[4]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const float u[4])
{
  const mxArray *y;
  static const int iv8[1] = { 0 };

  const mxArray *m4;
  static const int iv9[1] = { 4 };

  y = NULL;
  m4 = emlrtCreateNumericArray(1, iv8, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m4, (void *)u);
  emlrtSetDimensions((mxArray *)m4, iv9, 1);
  emlrtAssign(&y, m4);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : unsigned char
 */
static unsigned char k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  unsigned char ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 0U, 0);
  ret = *(unsigned char *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : unsigned char (*)[3]
 */
static unsigned char (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *src, const emlrtMsgIdentifier *msgId))[3]
{
  unsigned char (*ret)[3];
  int iv10[1];
  iv10[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "uint8", false, 1U, iv10);
  ret = (unsigned char (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float
 */
  static float m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  float ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 0U, 0);
  ret = *(float *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[9]
 */
static float (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  float (*ret)[9];
  int iv11[1];
  iv11[0] = 9;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv11);
  ret = (float (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[9]
 */
  static float (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[9]
{
  float (*ret)[9];
  int iv12[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv12[i] = 3;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv12);
  ret = (float (*)[9])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_AttitudeEKF_api.c
 *
 * [EOF]
 */
