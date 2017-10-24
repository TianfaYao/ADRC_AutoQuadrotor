/*
 * File: nav_h_ekf.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 11:01:12
 */

/* Include files */
#include "rt_nonfinite.h"
#include "nav_h_ekf.h"
#include "svd.h"

/* Variable Definitions */
 float X[3];
 float P[9];

/* Function Declarations */
static void b_eml_xscal(float a, float x[4], int ix0);
static float c_eml_div(float x, double y);
static float eml_div(double x, float y);
static void eml_xaxpy(float a, float y[4]);
static float eml_xdotc(const float x[4], const float y[4]);
static void eml_xgesvd(const float A[4], float U[4], float S[2], float V[4]);
static float eml_xnrm2(const float x[4]);
static void eml_xrot(float x[4], int ix0, int iy0, float c, float s);
static void eml_xrotg(float *a, float *b, float *c, float *s);
static void eml_xscal(float a, float x[4]);
static void eml_xswap(float x[4], int ix0, int iy0);

/* Function Definitions */

/*
 * Arguments    : float a
 *                float x[4]
 *                int ix0
 * Return Type  : void
 */
static void b_eml_xscal(float a, float x[4], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 1; k++) {
    x[k - 1] *= a;
  }
}

/*
 * Arguments    : float x
 *                double y
 * Return Type  : float
 */
static float c_eml_div(float x, double y)
{
  return x / (float)y;
}

/*
 * Arguments    : double x
 *                float y
 * Return Type  : float
 */
static float eml_div(double x, float y)
{
  return (float)x / y;
}

/*
 * Arguments    : float a
 *                float y[4]
 * Return Type  : void
 */
static void eml_xaxpy(float a, float y[4])
{
  int ix;
  int iy;
  int k;
  if (a == 0.0F) {
  } else {
    ix = 0;
    iy = 2;
    for (k = 0; k < 2; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : const float x[4]
 *                const float y[4]
 * Return Type  : float
 */
static float eml_xdotc(const float x[4], const float y[4])
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
  ix = 0;
  iy = 2;
  for (k = 0; k < 2; k++) {
    d += x[ix] * y[iy];
    ix++;
    iy++;
  }

  return d;
}

/*
 * Arguments    : const float A[4]
 *                float U[4]
 *                float S[2]
 *                float V[4]
 * Return Type  : void
 */
static void eml_xgesvd(const float A[4], float U[4], float S[2], float V[4])
{
  float b_A[4];
  float Vf[4];
  int kase;
  float nrm;
  float s[2];
  int m;
  float e[2];
  int q;
  float rt;
  float ztest;
  int iter;
  float tiny;
  float snorm;
  boolean_T exitg3;
  int qs;
  boolean_T exitg2;
  float f;
  float varargin_1[5];
  float mtmp;
  boolean_T exitg1;
  float sqds;
  for (kase = 0; kase < 4; kase++) {
    b_A[kase] = A[kase];
    Vf[kase] = 0.0F;
  }

  nrm = eml_xnrm2(A);
  if (nrm > 0.0F) {
    if (A[0] < 0.0F) {
      nrm = -nrm;
    }

    eml_xscal(eml_div(1.0, nrm), b_A);
    b_A[0]++;
    s[0] = -nrm;
  } else {
    s[0] = 0.0F;
  }

  if (s[0] != 0.0F) {
    eml_xaxpy(-b_eml_div(eml_xdotc(b_A, b_A), b_A[0]), b_A);
  }

  m = 1;
  s[1] = b_A[3];
  e[0] = b_A[2];
  e[1] = 0.0F;
  for (kase = 0; kase < 2; kase++) {
    U[kase] = b_A[kase];
    U[2 + kase] = 0.0F;
  }

  U[3] = 1.0F;
  if (s[0] != 0.0F) {
    eml_xaxpy(-b_eml_div(eml_xdotc(U, U), U[0]), U);
    for (kase = 0; kase < 2; kase++) {
      U[kase] = -U[kase];
    }

    U[0]++;
  } else {
    for (kase = 0; kase < 2; kase++) {
      U[kase] = 0.0F;
    }

    U[0] = 1.0F;
  }

  for (q = 1; q > -1; q += -1) {
    for (kase = 0; kase < 2; kase++) {
      Vf[kase + (q << 1)] = 0.0F;
    }

    Vf[q + (q << 1)] = 1.0F;
  }

  nrm = b_A[2];
  for (q = 0; q < 2; q++) {
    if (s[q] != 0.0F) {
      rt = (real32_T)fabs(s[q]);
      ztest = b_eml_div(s[q], rt);
      s[q] = rt;
      if (q + 1 < 2) {
        nrm = b_eml_div(nrm, ztest);
      }

      b_eml_xscal(ztest, U, (q << 1) + 1);
    }

    if ((q + 1 < 2) && (nrm != 0.0F)) {
      rt = (real32_T)fabs(nrm);
      ztest = b_eml_div(rt, nrm);
      nrm = rt;
      s[1] *= ztest;
      b_eml_xscal(ztest, Vf, 3);
    }

    e[0] = nrm;
  }

  iter = 0;
  tiny = b_eml_div(1.17549435E-38F, 1.1920929E-7F);
  snorm = 0.0F;
  for (kase = 0; kase < 2; kase++) {
    nrm = (real32_T)fabs(s[kase]);
    ztest = (real32_T)fabs(e[kase]);
    if ((nrm >= ztest) || rtIsNaNF(ztest)) {
    } else {
      nrm = ztest;
    }

    if ((snorm >= nrm) || rtIsNaNF(nrm)) {
    } else {
      snorm = nrm;
    }
  }

  while ((m + 1 > 0) && (!(iter >= 75))) {
    q = m;
    exitg3 = false;
    while (!(exitg3 || (q == 0))) {
      nrm = (real32_T)fabs(e[0]);
      if ((nrm <= 1.1920929E-7F * ((real32_T)fabs(s[0]) + (real32_T)fabs(s[1])))
          || (nrm <= tiny) || ((iter > 20) && (nrm <= 1.1920929E-7F * snorm))) {
        e[0] = 0.0F;
        exitg3 = true;
      } else {
        q = 0;
      }
    }

    if (q == m) {
      kase = 4;
    } else {
      qs = m + 1;
      kase = m + 1;
      exitg2 = false;
      while ((!exitg2) && (kase >= q)) {
        qs = kase;
        if (kase == q) {
          exitg2 = true;
        } else {
          nrm = 0.0F;
          if (kase < m + 1) {
            nrm = (real32_T)fabs(e[0]);
          }

          if (kase > q + 1) {
            nrm += (real32_T)fabs(e[0]);
          }

          ztest = (real32_T)fabs(s[kase - 1]);
          if ((ztest <= 1.1920929E-7F * nrm) || (ztest <= tiny)) {
            s[kase - 1] = 0.0F;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qs == q) {
        kase = 3;
      } else if (qs == m + 1) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[0];
      e[0] = 0.0F;
      qs = m;
      while (qs >= q + 1) {
        nrm = s[0];
        eml_xrotg(&nrm, &f, &ztest, &rt);
        s[0] = nrm;
        eml_xrot(Vf, 1, (m << 1) + 1, ztest, rt);
        qs = 0;
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (qs = q; qs + 1 <= m + 1; qs++) {
        eml_xrotg(&s[qs], &f, &ztest, &rt);
        f = -rt * e[qs];
        e[qs] *= ztest;
        eml_xrot(U, (qs << 1) + 1, ((q - 1) << 1) + 1, ztest, rt);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs(s[m]);
      varargin_1[1] = (real32_T)fabs(s[m - 1]);
      varargin_1[2] = (real32_T)fabs(e[m - 1]);
      varargin_1[3] = (real32_T)fabs(s[q]);
      varargin_1[4] = (real32_T)fabs(e[q]);
      kase = 1;
      mtmp = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        qs = 2;
        exitg1 = false;
        while ((!exitg1) && (qs < 6)) {
          kase = qs;
          if (!rtIsNaNF(varargin_1[qs - 1])) {
            mtmp = varargin_1[qs - 1];
            exitg1 = true;
          } else {
            qs++;
          }
        }
      }

      if (kase < 5) {
        while (kase + 1 < 6) {
          if (varargin_1[kase] > mtmp) {
            mtmp = varargin_1[kase];
          }

          kase++;
        }
      }

      f = b_eml_div(s[m], mtmp);
      nrm = b_eml_div(s[0], mtmp);
      ztest = b_eml_div(e[0], mtmp);
      sqds = b_eml_div(s[q], mtmp);
      rt = c_eml_div((nrm + f) * (nrm - f) + ztest * ztest, 2.0);
      nrm = f * ztest;
      nrm *= nrm;
      ztest = 0.0F;
      if ((rt != 0.0F) || (nrm != 0.0F)) {
        ztest = (real32_T)sqrt(rt * rt + nrm);
        if (rt < 0.0F) {
          ztest = -ztest;
        }

        ztest = b_eml_div(nrm, rt + ztest);
      }

      f = (sqds + f) * (sqds - f) + ztest;
      nrm = sqds * b_eml_div(e[q], mtmp);
      while (q + 1 <= m) {
        eml_xrotg(&f, &nrm, &ztest, &rt);
        f = ztest * s[0] + rt * e[0];
        e[0] = ztest * e[0] - rt * s[0];
        nrm = rt * s[1];
        s[1] *= ztest;
        eml_xrot(Vf, 1, 3, ztest, rt);
        s[0] = f;
        eml_xrotg(&s[0], &nrm, &ztest, &rt);
        f = ztest * e[0] + rt * s[1];
        s[1] = -rt * e[0] + ztest * s[1];
        nrm = rt * e[1];
        e[1] *= ztest;
        eml_xrot(U, 1, 3, ztest, rt);
        q = 1;
      }

      e[0] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        b_eml_xscal(-1.0F, Vf, (q << 1) + 1);
      }

      while ((q + 1 < 2) && (s[0] < s[1])) {
        rt = s[0];
        s[0] = s[1];
        s[1] = rt;
        eml_xswap(Vf, 1, 3);
        eml_xswap(U, 1, 3);
        q = 1;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (qs = 0; qs < 2; qs++) {
    S[qs] = s[qs];
    for (kase = 0; kase < 2; kase++) {
      V[kase + (qs << 1)] = Vf[kase + (qs << 1)];
    }
  }
}

/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
static float eml_xnrm2(const float x[4])
{
  float y;
  float scale;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 2; k++) {
    absxk = (real32_T)fabs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt(y);
}

/*
 * Arguments    : float x[4]
 *                int ix0
 *                int iy0
 *                float c
 *                float s
 * Return Type  : void
 */
static void eml_xrot(float x[4], int ix0, int iy0, float c, float s)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 2; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

/*
 * Arguments    : float *a
 *                float *b
 *                float *c
 *                float *s
 * Return Type  : void
 */
static void eml_xrotg(float *a, float *b, float *c, float *s)
{
  float roe;
  float absa;
  float absb;
  float scale;
  float ads;
  float bds;
  roe = *b;
  absa = (real32_T)fabs(*a);
  absb = (real32_T)fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    ads = 0.0F;
    scale = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    ads = scale * (real32_T)sqrt(ads * ads + bds * bds);
    if (roe < 0.0F) {
      ads = -ads;
    }

    *c = *a / ads;
    *s = *b / ads;
    if (absa > absb) {
      scale = *s;
    } else if (*c != 0.0F) {
      scale = 1.0F / *c;
    } else {
      scale = 1.0F;
    }
  }

  *a = ads;
  *b = scale;
}

/*
 * Arguments    : float a
 *                float x[4]
 * Return Type  : void
 */
static void eml_xscal(float a, float x[4])
{
  int k;
  for (k = 0; k < 2; k++) {
    x[k] *= a;
  }
}

/*
 * Arguments    : float x[4]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
static void eml_xswap(float x[4], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 2; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/*
 * 作者 ytf
 * 日期 2017年4月27日
 * 状态转移协方差矩阵
 * H 预测矩阵
 * R 观测噪声方差
 * F 状态转移矩阵
 * I
 * Z 测量矩阵
 * Arguments    : const float Q[9]
 *                const float H[6]
 *                const float R[4]
 *                const float F[9]
 *                const float I[9]
 *                const float Z[2]
 *                unsigned char up
 *                float *h
 *                float *v
 *                float *a
 * Return Type  : void
 */
void nav_h_ekf(const float Q[9], const float H[6], const float R[4], const float
               F[9], const float I[9], const float Z[2], unsigned char up, float
               *h, float *v, float *a)
{
  float K[3];
  int i;
  float b_F[9];
  int i0;
  int vcol;
  float tol;
  float y[6];
  float b_X[4];
  float b_H[6];
  float c_H[4];
  float V[4];
  float s[2];
  float U[4];
  int r;
  int ar;
  int ic;
  int ib;
  int ia;
  float b_K[6];
  float b_I[9];
  float c_X;
  float d_H[3];
  float b_V;
  float S;
  float b_U;

  /*  状态变量 */
  /*  状态协方差矩阵 */
  for (i = 0; i < 3; i++) {
    K[i] = X[i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    X[i0] = 0.0F;
    for (i = 0; i < 3; i++) {
      X[i0] += F[i0 + 3 * i] * K[i];
    }

    for (i = 0; i < 3; i++) {
      b_F[i0 + 3 * i] = 0.0F;
      for (vcol = 0; vcol < 3; vcol++) {
        b_F[i0 + 3 * i] += F[i0 + 3 * vcol] * P[vcol + 3 * i];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      tol = 0.0F;
      for (vcol = 0; vcol < 3; vcol++) {
        tol += b_F[i0 + 3 * vcol] * F[i + 3 * vcol];
      }

      P[i0 + 3 * i] = tol + Q[i0 + 3 * i];
    }
  }

  if (up == 1) {
    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 2; i++) {
        y[i0 + 3 * i] = 0.0F;
        for (vcol = 0; vcol < 3; vcol++) {
          y[i0 + 3 * i] += P[i0 + 3 * vcol] * H[i + (vcol << 1)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      b_X[i0] = 0.0F;
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i = 0; i < 3; i++) {
        b_H[i0 + (i << 1)] = 0.0F;
        for (vcol = 0; vcol < 3; vcol++) {
          b_H[i0 + (i << 1)] += H[i0 + (vcol << 1)] * P[vcol + 3 * i];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i = 0; i < 2; i++) {
        tol = 0.0F;
        for (vcol = 0; vcol < 3; vcol++) {
          tol += b_H[i0 + (vcol << 1)] * H[i + (vcol << 1)];
        }

        c_H[i0 + (i << 1)] = tol + R[i0 + (i << 1)];
      }
    }

    eml_xgesvd(c_H, U, s, V);
    for (i0 = 0; i0 < 4; i0++) {
      c_H[i0] = 0.0F;
    }

    for (i = 0; i < 2; i++) {
      c_H[i + (i << 1)] = s[i];
    }

    tol = 2.0F * c_H[0] * 1.1920929E-7F;
    r = 0;
    i = 0;
    while ((i + 1 < 3) && (c_H[i + (i << 1)] > tol)) {
      r++;
      i++;
    }

    if (r > 0) {
      vcol = 0;
      for (ar = 0; ar + 1 <= r; ar++) {
        tol = 1.0F / c_H[ar + (ar << 1)];
        for (i = vcol; i + 1 <= vcol + 2; i++) {
          V[i] *= tol;
        }

        vcol += 2;
      }

      for (i = 0; i < 4; i += 2) {
        for (ic = i; ic + 1 <= i + 2; ic++) {
          b_X[ic] = 0.0F;
        }
      }

      vcol = -1;
      for (i = 0; i < 4; i += 2) {
        ar = 0;
        vcol++;
        i0 = (vcol + ((r - 1) << 1)) + 1;
        for (ib = vcol; ib + 1 <= i0; ib += 2) {
          if (U[ib] != 0.0F) {
            ia = ar;
            for (ic = i; ic + 1 <= i + 2; ic++) {
              ia++;
              b_X[ic] += U[ib] * V[ia - 1];
            }
          }

          ar += 2;
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 2; i++) {
        b_K[i0 + 3 * i] = 0.0F;
        for (vcol = 0; vcol < 2; vcol++) {
          b_K[i0 + 3 * i] += y[i0 + 3 * vcol] * b_X[vcol + (i << 1)];
        }
      }
    }

    /* 增益 */
    for (i0 = 0; i0 < 2; i0++) {
      tol = 0.0F;
      for (i = 0; i < 3; i++) {
        tol += H[i0 + (i << 1)] * X[i];
      }

      s[i0] = Z[i0] - tol;
    }

    for (i0 = 0; i0 < 3; i0++) {
      tol = 0.0F;
      for (i = 0; i < 2; i++) {
        tol += b_K[i0 + 3 * i] * s[i];
      }

      X[i0] += tol;
    }

    for (i0 = 0; i0 < 9; i0++) {
      b_F[i0] = P[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 3; i++) {
        tol = 0.0F;
        for (vcol = 0; vcol < 2; vcol++) {
          tol += b_K[i0 + 3 * vcol] * H[vcol + (i << 1)];
        }

        b_I[i0 + 3 * i] = I[i0 + 3 * i] - tol;
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 3; i++) {
        P[i0 + 3 * i] = 0.0F;
        for (vcol = 0; vcol < 3; vcol++) {
          P[i0 + 3 * i] += b_I[i0 + 3 * vcol] * b_F[vcol + 3 * i];
        }
      }
    }
  }

  if (up == 0) {
    c_X = 0.0F;
    tol = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      K[i0] = 0.0F;
      for (i = 0; i < 3; i++) {
        K[i0] += P[i0 + 3 * i] * H[1 + (i << 1)];
      }

      d_H[i0] = 0.0F;
      for (i = 0; i < 3; i++) {
        d_H[i0] += H[1 + (i << 1)] * P[i + 3 * i0];
      }

      tol += d_H[i0] * H[1 + (i0 << 1)];
    }

    svd(tol + R[3], &b_U, &S, &b_V);
    r = 0;
    if (!(S > S * 1.1920929E-7F)) {
    } else {
      r = 1;
    }

    if (r > 0) {
      tol = 1.0F / S;
      i = 1;
      while (i <= 1) {
        b_V *= tol;
        i = 2;
      }

      if (b_U != 0.0F) {
        c_X = b_U * b_V;
      }
    }

    /* 增益 */
    tol = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      tol += H[1 + (i0 << 1)] * X[i0];
      K[i0] *= c_X;
    }

    tol = Z[1] - tol;
    for (i0 = 0; i0 < 3; i0++) {
      X[i0] += K[i0] * tol;
    }

    for (i0 = 0; i0 < 9; i0++) {
      b_F[i0] = P[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 3; i++) {
        b_I[i0 + 3 * i] = I[i0 + 3 * i] - K[i0] * H[1 + (i << 1)];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 3; i++) {
        P[i0 + 3 * i] = 0.0F;
        for (vcol = 0; vcol < 3; vcol++) {
          P[i0 + 3 * i] += b_I[i0 + 3 * vcol] * b_F[vcol + 3 * i];
        }
      }
    }
  }

  *h = X[0];
  *v = X[1];
  *a = X[2];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_h_ekf_init(void)
{
  int i;
  static const int  iv0[9] = { 100, 0, 0, 0, 100, 0, 0, 0, 100 };
  for (i = 0; i < 9; i++) {
    P[i] = iv0[i];
  }

  for (i = 0; i < 3; i++) {
    X[i] = 0.0F;
  }
}

/*
 * File trailer for nav_h_ekf.c
 *
 * [EOF]
 */
