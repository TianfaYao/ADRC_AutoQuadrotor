#include "uav_matrix.h"

//*******************************************************************************/
//3x3矩阵相加
int MatAdd3x3(fmat3x3 * src1, fmat3x3 * src2, fmat3x3 * dest)
{
    int i, j;

    /*
       sum.data[0][0] = src1->data[0][0] + src2->data[0][0];
       sum.data[0][1] = src1->data[0][1] + src2->data[0][1];
       sum.data[0][2] = src1->data[0][2] + src2->data[0][2];

       sum.data[1][0] = src1->data[1][0] + src2->data[1][0];
       sum.data[1][1] = src1->data[1][1] + src2->data[1][1];
       sum.data[1][2] = src1->data[1][2] + src2->data[1][2];

       sum.data[2][0] = src1->data[2][0] + src2->data[2][0];
       sum.data[2][1] = src1->data[2][1] + src2->data[2][1];
       sum.data[2][2] = src1->data[2][2] + src2->data[2][2];
     */

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dest->data[i][j] = src1->data[i][j] + src2->data[i][j];
        }
    }

    return 1;
}

/*******************************************************************************
* Function Name  : MatMult3x3
* Input          : fmat3x3* src1, fmat3x3* src2
* Output         : fmat3x3* dest
* Return         : 0 if failed, 1 if success
* Description    :

Performs the operation: dest = src1*src2;

'dest' can be the same as either src1 or src2
*******************************************************************************/
//3x3矩阵相乘
int MatMult3x3(fmat3x3 * src1, fmat3x3 * src2, fmat3x3 * dest)
{
    int i, j;
    fmat3x3 result;

    // First row
    result.data[0][0] = src1->data[0][0] * src2->data[0][0] + src1->data[0][1] * src2->data[1][0] + src1->data[0][2] * src2->data[2][0];
    result.data[0][1] = src1->data[0][0] * src2->data[0][1] + src1->data[0][1] * src2->data[1][1] + src1->data[0][2] * src2->data[2][1];
    result.data[0][2] = src1->data[0][0] * src2->data[0][2] + src1->data[0][1] * src2->data[1][2] + src1->data[0][2] * src2->data[2][2];

    // Second row
    result.data[1][0] = src1->data[1][0] * src2->data[0][0] + src1->data[1][1] * src2->data[1][0] + src1->data[1][2] * src2->data[2][0];
    result.data[1][1] = src1->data[1][0] * src2->data[0][1] + src1->data[1][1] * src2->data[1][1] + src1->data[1][2] * src2->data[2][1];
    result.data[1][2] = src1->data[1][0] * src2->data[0][2] + src1->data[1][1] * src2->data[1][2] + src1->data[1][2] * src2->data[2][2];

    // Third row
    result.data[2][0] = src1->data[2][0] * src2->data[0][0] + src1->data[2][1] * src2->data[1][0] + src1->data[2][2] * src2->data[2][0];
    result.data[2][1] = src1->data[2][0] * src2->data[0][1] + src1->data[2][1] * src2->data[1][1] + src1->data[2][2] * src2->data[2][1];
    result.data[2][2] = src1->data[2][0] * src2->data[0][2] + src1->data[2][1] * src2->data[1][2] + src1->data[2][2] * src2->data[2][2];

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dest->data[i][j] = result.data[i][j];
        }
    }

    return 1;
}

/*******************************************************************************
* Function Name  : MatInv3x3
* Input          : fmat3x3* src1, fmat3x3* src2
* Output         : fmat3x3* dest
* Return         : 0 if failed, 1 if success
* Description    :

Computes the inverse of the 3x3 matrix in src
*******************************************************************************/
//3x3矩阵求逆
int MatInv3x3(fmat3x3 * src, fmat3x3 * dest)
{
    fmat3x3 inverse;
    int i, j;

    float det = MatDet3x3(src);

    // Invert the matrix
    /*
       | a11 a12 a13 |-1             |   a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13  |
       | a21 a22 a23 |    =  1/DET * | -(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13) |
       | a31 a32 a33 |               |   a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12  |

       | a00 a01 a02 |-1             |   a22a11-a21a12  -(a22a01-a21a02)   a12a01-a11a02  |
       | a10 a11 a12 |    =  1/DET * | -(a22a10-a20a12)   a22a00-a20a02  -(a12a00-a10a02) |
       | a20 a21 a22 |               |   a21a10-a20a11  -(a21a00-a20a01)   a11a00-a10a01  |
     */

    // Row 1
//       inverse.data[0][0] = (a22a11-a21a12)/det;
    inverse.data[0][0] = (src->data[2][2] * src->data[1][1] - src->data[2][1] * src->data[1][2]) / det;
//       inverse.data[0][1] = -(a22a01-a21a02)/det;
    inverse.data[0][1] = -(src->data[2][2] * src->data[0][1] - src->data[2][1] * src->data[0][2]) / det;
//       inverse.data[0][2] = (a12a01-a11a02)/det;
    inverse.data[0][2] = (src->data[1][2] * src->data[0][1] - src->data[1][1] * src->data[0][2]) / det;

    // Row 2
//       inverse.data[1][0] = -(a22a10-a20a12)/det;
    inverse.data[1][0] = -(src->data[2][2] * src->data[1][0] - src->data[2][0] * src->data[1][2]) / det;
//       inverse.data[1][1] = (a22a00-a20a02)/det;
    inverse.data[1][1] = (src->data[2][2] * src->data[0][0] - src->data[2][0] * src->data[0][2]) / det;
//       inverse.data[1][2] = -(a12a00-a10a02)/det;
    inverse.data[1][2] = -(src->data[1][2] * src->data[0][0] - src->data[1][0] * src->data[0][2]) / det;

    // Row 3
//       inverse.data[2][0] = (a21a10-a20a11)/det;
    inverse.data[2][0] = (src->data[2][1] * src->data[1][0] - src->data[2][0] * src->data[1][1]) / det;
//       inverse.data[2][1] = -(a21a00-a20a01)/det;
    inverse.data[2][1] = -(src->data[2][1] * src->data[0][0] - src->data[2][0] * src->data[0][1]) / det;
//       inverse.data[2][2] = (a11a00-a10a01)/det;
    inverse.data[2][2] = (src->data[1][1] * src->data[0][0] - src->data[1][0] * src->data[0][1]) / det;

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dest->data[i][j] = inverse.data[i][j];
        }
    }

    return 1;
}

/*******************************************************************************
* Function Name  : MatVectMult3
* Input          : fmat3x3* matrix, fvect3x1* vector
* Output         : fmat3x3* dest
* Return         : 0 if failed, 1 if success
* Description    :

Performs the operation: dest = matrix*vector
*******************************************************************************/
//3X3矩阵和一个列向量相乘
int MatVectMult3(fmat3x3 * matrix, fvect3x1 * vector, fvect3x1 * dest)
{
    fvect3x1 result;
    int i;

    result.data[0] = matrix->data[0][0] * vector->data[0] + matrix->data[0][1] * vector->data[1] + matrix->data[0][2] * vector->data[2];
    result.data[1] = matrix->data[1][0] * vector->data[0] + matrix->data[1][1] * vector->data[1] + matrix->data[1][2] * vector->data[2];
    result.data[2] = matrix->data[2][0] * vector->data[0] + matrix->data[2][1] * vector->data[1] + matrix->data[2][2] * vector->data[2];

    for (i = 0; i < 3; i++) {
        dest->data[i] = result.data[i];
    }

    return 1;
}

/*******************************************************************************
* Function Name  : MatDet3x3
* Input          : fmat3x3* src
* Output         : None
* Return         : The determinant of the matrix
* Description    :

Computes the determinant of the matrix in 'src'
*******************************************************************************/
//矩阵行列式
float MatDet3x3(fmat3x3 * src)
{
    // det = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)
    // det = a00(a22a11-a21a12)-a10(a22a01-a21a02)+a20(a12a01-a11a02)

    return src->data[0][0] * (src->data[2][2] * src->data[1][1] - src->data[2][1] * src->data[1][2])
        - src->data[1][0] * (src->data[2][2] * src->data[0][1] - src->data[2][1] * src->data[0][2])
        + src->data[2][0] * (src->data[1][2] * src->data[0][1] - src->data[1][1] * src->data[0][2]);
}

/*******************************************************************************
* Function Name  : MatTrans3x3
* Input          : fmat3x3* src
* Output         : fmat3x3* dest
* Return         : 0 if failed, 1 if success
* Description    :

Transposes the matrix in 'src', stores the result in 'dest'
*******************************************************************************/
//矩阵转置
int MatTrans3x3(fmat3x3 * src, fmat3x3 * dest)
{
    fmat3x3 temp;
    int i, j;

    temp.data[0][0] = src->data[0][0];
    temp.data[0][1] = src->data[1][0];
    temp.data[0][2] = src->data[2][0];

    temp.data[1][0] = src->data[0][1];
    temp.data[1][1] = src->data[1][1];
    temp.data[1][2] = src->data[2][1];

    temp.data[2][0] = src->data[0][2];
    temp.data[2][1] = src->data[1][2];
    temp.data[2][2] = src->data[2][2];

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            dest->data[i][j] = temp.data[i][j];
        }
    }

    return 1;
}

/*******************************************************************************
* Function Name  : ScalarMatMult3x3
* Input          : float scal, fmat3x3* src
* Output         : fmat3x3* dest
* Return         : 0 if failed, 1 if success
* Description    :

Multiplies the elements in src by scal and stores the result in dest

*******************************************************************************/
//矩阵相除
int ScalarMatMult3x3(float scal, fmat3x3 * src, fmat3x3 * dest)
{
    dest->data[0][0] = src->data[0][0] * scal;
    dest->data[0][1] = src->data[0][1] * scal;
    dest->data[0][2] = src->data[0][2] * scal;

    dest->data[1][0] = src->data[1][0] * scal;
    dest->data[1][1] = src->data[1][1] * scal;
    dest->data[1][2] = src->data[1][2] * scal;

    dest->data[2][0] = src->data[2][0] * scal;
    dest->data[2][1] = src->data[2][1] * scal;
    dest->data[2][2] = src->data[2][2] * scal;

    return 1;
}

/*******************************************************************************
* Function Name  : CreateIdentity3x3
* Input          : None
* Output         : fmat3x3* dest
* Return         : Void
* Description    :

Creates a 3x3 identity matrix

*******************************************************************************/
//创建对角为一矩阵
void CreateIdentity3x3(fmat3x3 * dest)
{
    MatZero3x3(dest);

    dest->data[0][0] = 1;
    dest->data[1][1] = 1;
    dest->data[2][2] = 1;
}

/*******************************************************************************
* Function Name  : MatZero3x3
* Input          : None
* Output         : fmat3x3* dest
* Return         : Void
* Description    :

Zeros out all the entries in dest

*******************************************************************************/
//创建0
void MatZero3x3(fmat3x3 * dest)
{
    dest->data[0][0] = 0;
    dest->data[0][1] = 0;
    dest->data[0][2] = 0;

    dest->data[1][0] = 0;
    dest->data[1][1] = 0;
    dest->data[1][2] = 0;

    dest->data[2][0] = 0;
    dest->data[2][1] = 0;
    dest->data[2][2] = 0;
}

/*******************************************************************************
* Function Name  : MatCopy3x3
* Input          : fmat3x3* src
* Output         : fmat3x3* dest
* Return         : Void
* Description    :

Copies the data in src to dest.

*******************************************************************************/
void MatCopy3x3(fmat3x3 * src, fmat3x3 * dest)
{
    dest->data[0][0] = src->data[0][0];
    dest->data[0][1] = src->data[0][1];
    dest->data[0][2] = src->data[0][2];

    dest->data[1][0] = src->data[1][0];
    dest->data[1][1] = src->data[1][1];
    dest->data[1][2] = src->data[1][2];

    dest->data[2][0] = src->data[2][0];
    dest->data[2][1] = src->data[2][1];
    dest->data[2][2] = src->data[2][2];
}
