#ifndef UAV_MATRIX_H
#define UAV_MATRIX_H
#include "stm32f4xx.h"

// Structure for holding 3x3 matrices
typedef struct _fmat3x3 {
    float data[3][3];
} fmat3x3;

// Structure for holding 3-element vectors
typedef struct _fvect3x1 {
    float data[3];
} fvect3x1;

// Function declarations
int MatAdd3x3(fmat3x3 * src1, fmat3x3 * src2, fmat3x3 * dest);
int MatMult3x3(fmat3x3 * src1, fmat3x3 * src2, fmat3x3 * dest);
int MatInv3x3(fmat3x3 * src, fmat3x3 * dest);
int MatVectMult3(fmat3x3 * matrix, fvect3x1 * vector, fvect3x1 * dest);
float MatDet3x3(fmat3x3 * src);
int MatTrans3x3(fmat3x3 * src, fmat3x3 * dest);
int ScalarMatMult3x3(float T, fmat3x3 * src, fmat3x3 * dest);
void CreateIdentity3x3(fmat3x3 * dest);
void MatZero3x3(fmat3x3 * dest);
void MatCopy3x3(fmat3x3 * src, fmat3x3 * dest);


#endif