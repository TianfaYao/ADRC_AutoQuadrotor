#include "cal_matrix3.h"
#include "cal_math.h"
#define HALF_SQRT_2 0.70710678118654757

//欧拉角转余弦矩阵
template <typename T>
void Matrix3<T>::from_euler(const Vector3<T> &euler)
{
	  float sinx = sinf(euler.x);
    float cosx = cosf(euler.x);
	
    float siny = sinf(euler.y);
    float cosy = cosf(euler.y);
	
    float sinz = sinf(euler.z);
    float cosz = cosf(euler.z);

    a.x = cosy * cosz;
    a.y = (sinx * siny * cosz) + (cosx * sinz);
    a.z = -(cosx * siny * cosz) + (sinx * sinz);
    b.x = -cosy * sinz;
    b.y = -(sinx * siny * sinz) + (cosx * cosz);
    b.z = (cosx * siny * sinz) + (sinx * cosz);
    c.x = siny;
    c.y = -sinx * cosy;
    c.z = cosx * cosy;
	
}






//余弦矩阵计算欧拉角
template <typename T>
void Matrix3<T>::to_euler(float *roll, float *pitch, float *yaw)
{
    if (pitch != NULL) {
        //*pitch = degrees(-safe_asin(c.x));
				*pitch = degrees(atan2f(-c.x,sqrtf(c.y*c.y + c.z*c.z)));
    }
    if (roll != NULL) {
        *roll = degrees(atan2f(c.y, c.z));
    }
    if (yaw != NULL) {
        *yaw = -degrees(atan2f(b.x, a.x));
    }
}

//使用角速度向量计算矩阵的一次旋转
template <typename T>
void Matrix3<T>::rotate(const Vector3<T> &g)
{
    Matrix3f temp_matrix;
    temp_matrix.a.x = a.y * g.z - a.z * g.y;
    temp_matrix.a.y = a.z * g.x - a.x * g.z;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
	
    temp_matrix.b.x = b.y * g.z - b.z * g.y;
    temp_matrix.b.y = b.z * g.x - b.x * g.z;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
	
    temp_matrix.c.x = c.y * g.z - c.z * g.y;
    temp_matrix.c.y = c.z * g.x - c.x * g.z;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}

//使用角速度向量计算矩阵的一次旋转
template <typename T>
void Matrix3<T>::rotateXY(const Vector3<T> &g)
{
    Matrix3f temp_matrix;
    temp_matrix.a.x = -a.z * g.y;
    temp_matrix.a.y = a.z * g.x;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
	
    temp_matrix.b.x = -b.z * g.y;
    temp_matrix.b.y = b.z * g.x;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
	
    temp_matrix.c.x = -c.z * g.y;
    temp_matrix.c.y = c.z * g.x;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}

//矩阵乘以一个向量
template <typename T>
Vector3<T> Matrix3<T>::operator *(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z,
                      c.x * v.x + c.y * v.y + c.z * v.z);
}

//矩阵的转置乘以一个向量
template <typename T>
Vector3<T> Matrix3<T>::mul_transpose(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + b.x * v.y + c.x * v.z,
                      a.y * v.x + b.y * v.y + c.y * v.z,
                      a.z * v.x + b.z * v.y + c.z * v.z);
}

//和另一个3阶方阵相乘
template <typename T>
Matrix3<T> Matrix3<T>::operator *(const Matrix3<T> &m) const
{
    Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
                                a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
                                a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
                     Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
                                b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
                                b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
                     Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
                                c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
                                c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
    return temp;
}

//矩阵转置
template <typename T>
Matrix3<T> Matrix3<T>::transposed(void) const
{
    return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
                      Vector3<T>(a.y, b.y, c.y),
                      Vector3<T>(a.z, b.z, c.z));
}

//矩阵元素置零
template <typename T>
void Matrix3<T>::zero(void)
{
    a.x = a.y = a.z = 0;
    b.x = b.y = b.z = 0;
    c.x = c.y = c.z = 0;
}

//只给浮点型定义了
template void Matrix3<float>::zero(void);
template void Matrix3<float>::rotate(const Vector3<float> &g);
template void Matrix3<float>::rotateXY(const Vector3<float> &g);
template void Matrix3<float>::from_euler(const Vector3<float> &euler);
template void Matrix3<float>::to_euler(float *roll, float *pitch, float *yaw);
template Vector3<float> Matrix3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Matrix3<float>::mul_transpose(const Vector3<float> &v) const;
template Matrix3<float> Matrix3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Matrix3<float>::transposed(void) const;




