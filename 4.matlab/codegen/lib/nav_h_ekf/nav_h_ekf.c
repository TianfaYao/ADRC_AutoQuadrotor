/*
 * File: nav_h_ekf.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Jul-2017 03:24:26
 */

/* Include files */
#include "rt_nonfinite.h"
#include "nav_h_ekf.h"

/* Variable Definitions */
static float X[4];
static float P[16];

/* Function Definitions */

/*
 * 作者 ytf
 * 日期 2017年4月27日
 * 状态转移协方差矩阵
 * H 预测矩阵
 * R 观测噪声方差
 * F 状态转移矩阵
 * I
 * Z 测量矩阵
 * Arguments    : const float Q[16]
 *                const float H[8]
 *                const float R[4]
 *                const float F[16]
 *                const float I[16]
 *                const float Z[2]
 *                unsigned char up
 *                float *h
 *                float *v
 *                float *a
 *                float *a_a
 * Return Type  : void
 */
void nav_h_ekf(const float Q[16], const float H[8], const float R[4], const
               float F[16], const float I[16], const float Z[2], unsigned char
               up, float *h, float *v, float *a, float *a_a)
{
  float K[4];
  int i;
  float b_F[16];
  int r2;
  int k;
  float a21;
  float y[8];
  float b_K[8];
  float a22;
  float b_Z[2];
  float B[16];

  /*  状态变量 */
  /*  状态协方差矩阵 */
  for (i = 0; i < 4; i++) {
    K[i] = X[i];
  }

  for (i = 0; i < 4; i++) {
    X[i] = 0.0F;
    for (r2 = 0; r2 < 4; r2++) {
      X[i] += F[i + (r2 << 2)] * K[r2];
    }

    for (r2 = 0; r2 < 4; r2++) {
      b_F[i + (r2 << 2)] = 0.0F;
      for (k = 0; k < 4; k++) {
        b_F[i + (r2 << 2)] += F[i + (k << 2)] * P[k + (r2 << 2)];
      }
    }
  }

  for (i = 0; i < 4; i++) {
    for (r2 = 0; r2 < 4; r2++) {
      a21 = 0.0F;
      for (k = 0; k < 4; k++) {
        a21 += b_F[i + (k << 2)] * F[r2 + (k << 2)];
      }

      P[i + (r2 << 2)] = a21 + Q[i + (r2 << 2)];
    }
  }

  if (up == 1) {
    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < 2; r2++) {
        y[i + (r2 << 2)] = 0.0F;
        for (k = 0; k < 4; k++) {
          y[i + (r2 << 2)] += P[i + (k << 2)] * H[r2 + (k << 1)];
        }
      }
    }

    for (i = 0; i < 2; i++) {
      for (r2 = 0; r2 < 4; r2++) {
        b_K[i + (r2 << 1)] = 0.0F;
        for (k = 0; k < 4; k++) {
          b_K[i + (r2 << 1)] += H[i + (k << 1)] * P[k + (r2 << 2)];
        }
      }
    }

    for (i = 0; i < 2; i++) {
      for (r2 = 0; r2 < 2; r2++) {
        a21 = 0.0F;
        for (k = 0; k < 4; k++) {
          a21 += b_K[i + (k << 1)] * H[r2 + (k << 1)];
        }

        K[i + (r2 << 1)] = a21 + R[i + (r2 << 1)];
      }
    }

    if ((real32_T)fabs(K[1]) > (real32_T)fabs(K[0])) {
      i = 1;
      r2 = 0;
    } else {
      i = 0;
      r2 = 1;
    }

    a21 = K[r2] / K[i];
    a22 = K[2 + r2] - a21 * K[2 + i];
    for (k = 0; k < 4; k++) {
      b_K[k + (i << 2)] = y[k] / K[i];
      b_K[k + (r2 << 2)] = (y[4 + k] - b_K[k + (i << 2)] * K[2 + i]) / a22;
      b_K[k + (i << 2)] -= b_K[k + (r2 << 2)] * a21;
    }

    /* 增益 */
    for (i = 0; i < 2; i++) {
      a21 = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        a21 += H[i + (r2 << 1)] * X[r2];
      }

      b_Z[i] = Z[i] - a21;
    }

    for (i = 0; i < 4; i++) {
      a21 = 0.0F;
      for (r2 = 0; r2 < 2; r2++) {
        a21 += b_K[i + (r2 << 2)] * b_Z[r2];
      }

      X[i] += a21;
    }

    memcpy(&B[0], &P[0], sizeof(float) << 4);
    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < 4; r2++) {
        a21 = 0.0F;
        for (k = 0; k < 2; k++) {
          a21 += b_K[i + (k << 2)] * H[k + (r2 << 1)];
        }

        b_F[i + (r2 << 2)] = I[i + (r2 << 2)] - a21;
      }
    }

    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < 4; r2++) {
        P[i + (r2 << 2)] = 0.0F;
        for (k = 0; k < 4; k++) {
          P[i + (r2 << 2)] += b_F[i + (k << 2)] * B[k + (r2 << 2)];
        }
      }
    }
  }

  if (up == 0) {
    a21 = 0.0F;
    for (i = 0; i < 4; i++) {
      K[i] = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        K[i] += H[1 + (r2 << 1)] * P[r2 + (i << 2)];
      }

      a21 += K[i] * H[1 + (i << 1)];
    }

    a22 = a21 + R[3];
    for (i = 0; i < 4; i++) {
      a21 = 0.0F;
      for (r2 = 0; r2 < 4; r2++) {
        a21 += P[i + (r2 << 2)] * H[1 + (r2 << 1)];
      }

      K[i] = a21 / a22;
    }

    /* 增益 */
    a21 = 0.0F;
    for (i = 0; i < 4; i++) {
      a21 += H[1 + (i << 1)] * X[i];
    }

    a21 = Z[1] - a21;
    for (i = 0; i < 4; i++) {
      X[i] += K[i] * a21;
    }

    memcpy(&B[0], &P[0], sizeof(float) << 4);
    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < 4; r2++) {
        b_F[i + (r2 << 2)] = I[i + (r2 << 2)] - K[i] * H[1 + (r2 << 1)];
      }
    }

    for (i = 0; i < 4; i++) {
      for (r2 = 0; r2 < 4; r2++) {
        P[i + (r2 << 2)] = 0.0F;
        for (k = 0; k < 4; k++) {
          P[i + (r2 << 2)] += b_F[i + (k << 2)] * B[k + (r2 << 2)];
        }
      }
    }
  }

  *h = X[0];
  *v = X[1];
  *a = X[2];
  *a_a = X[3];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void nav_h_ekf_init(void)
{
  static const float fv0[16] = { 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 100.0F };

  int i;
  memcpy(&P[0], &fv0[0], sizeof(float) << 4);
  for (i = 0; i < 4; i++) {
    X[i] = 0.0F;
  }
}

/*
 * File trailer for nav_h_ekf.c
 *
 * [EOF]
 */
