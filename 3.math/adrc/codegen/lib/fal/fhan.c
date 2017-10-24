/*
 * File: fhan.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Jun-2017 11:24:40
 */

/* Include files */
#include "rt_nonfinite.h"
#include "fal.h"
#include "fhan.h"

/* Function Definitions */

/*
 * % °ô°ô·¨  bang_bang
 * x1(k+1)=x1(k)+h*x2;
 * x2(k+1)=x2(k)-r*u
 * %
 *  d=r*h^2;
 *  a0==h*x2;
 *  y=x1+a0;
 *  a1=sqrt(d*(d+8*abs(y)));
 *  a2=a0+sign(y)(a1-d)/2;
 *  Sy=(sign(y+d)-sign(y-d))/2;fsg
 *  a=(a0+y-a2)*Sy+a2;
 *  Sa=(sign(a+d)-sign(a-d))/2;
 *  fhan=-r(a/d-sign(a))*Sa-r*sign(a);
 *
 *                ÀëÉ¢×´Ì¬
 *  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 *  fh=fhan(x1(k)-v(k),x2(k),r,h);
 *  x1(k+1)=x1(k)+h*x2(k);
 *  x2(k+1)=x2(k)+h*fh;
 * %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 * %
 * Arguments    : float x1
 *                float x2
 *                float r
 *                float h
 * Return Type  : float
 */
float fhan(float x1, float x2, float r, float h)
{
  float d;
  float a0;
  float y;
  float b_y;
  float a2;
  float x;
  float b_x;
  float c_x;
  float d_x;
  float b_a0;
  float e_x;
  float f_x;
  float c_a0;
  d = r * (h * h);
  a0 = h * x2;
  y = x1 + a0;
  if (y < 0.0F) {
    b_y = -1.0F;
  } else if (y > 0.0F) {
    b_y = 1.0F;
  } else if (y == 0.0F) {
    b_y = 0.0F;
  } else {
    b_y = y;
  }

  a2 = a0 + b_y * ((real32_T)sqrt(d * (d + 8.0F * (real32_T)fabs(y))) - d) /
    2.0F;
  x = y + d;
  b_x = y - d;
  if (x < 0.0F) {
    c_x = -1.0F;
  } else if (x > 0.0F) {
    c_x = 1.0F;
  } else if (x == 0.0F) {
    c_x = 0.0F;
  } else {
    c_x = x;
  }

  if (b_x < 0.0F) {
    d_x = -1.0F;
  } else if (b_x > 0.0F) {
    d_x = 1.0F;
  } else if (b_x == 0.0F) {
    d_x = 0.0F;
  } else {
    d_x = b_x;
  }

  a0 = ((a0 + y) - a2) * ((c_x - d_x) / 2.0F) + a2;
  x = a0 + d;
  b_x = a0 - d;
  if (a0 < 0.0F) {
    b_a0 = -1.0F;
  } else if (a0 > 0.0F) {
    b_a0 = 1.0F;
  } else if (a0 == 0.0F) {
    b_a0 = 0.0F;
  } else {
    b_a0 = a0;
  }

  if (x < 0.0F) {
    e_x = -1.0F;
  } else if (x > 0.0F) {
    e_x = 1.0F;
  } else if (x == 0.0F) {
    e_x = 0.0F;
  } else {
    e_x = x;
  }

  if (b_x < 0.0F) {
    f_x = -1.0F;
  } else if (b_x > 0.0F) {
    f_x = 1.0F;
  } else if (b_x == 0.0F) {
    f_x = 0.0F;
  } else {
    f_x = b_x;
  }

  if (a0 < 0.0F) {
    c_a0 = -1.0F;
  } else if (a0 > 0.0F) {
    c_a0 = 1.0F;
  } else if (a0 == 0.0F) {
    c_a0 = 0.0F;
  } else {
    c_a0 = a0;
  }

  return -r * (a0 / d - b_a0) * ((e_x - f_x) / 2.0F) - r * c_a0;
}

/*
 * File trailer for fhan.c
 *
 * [EOF]
 */
