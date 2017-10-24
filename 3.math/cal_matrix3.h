#ifndef CAL_MATRIX3_H
#define CAL_MATRIX3_H

#include "cal_vector3.h"
// 元素类型为T的3x3矩阵
template <typename T>
class Matrix3 {
public:

    //矩阵的行向量
    Vector3<T>        a, b, c;

    //空构造函数
    Matrix3<T>() {
    }

    //赋值构造函数
    Matrix3<T>(const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0) : a(a0), b(b0), c(c0) {
    }

    //赋值构造函数
    Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz) : a(ax,ay,az), b(bx,by,bz), c(cx,cy,cz) {
    }

    //函数调用操作符
    void operator        () (const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0)
    {
        a = a0; b = b0; c = c0;
    }

    //测试是否相等
    bool operator        == (const Matrix3<T> &m)
    {
        return (a==m.a && b==m.b && c==m.c);
    }

    //测试是否不相等
    bool operator        != (const Matrix3<T> &m)
    {
        return (a!=m.a || b!=m.b || c!=m.c);
    }

    //取反
    Matrix3<T> operator        - (void) const
    {
        return Matrix3<T>(-a,-b,-c);
    }

    //相加
    Matrix3<T> operator        + (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a+m.a, b+m.b, c+m.c);
    }
    Matrix3<T> &operator        += (const Matrix3<T> &m)
    {
        return *this = *this + m;
    }

    //相减
    Matrix3<T> operator        - (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a-m.a, b-m.b, c-m.c);
    }
    Matrix3<T> &operator        -= (const Matrix3<T> &m)
    {
        return *this = *this - m;
    }

    //标量操作
    Matrix3<T> operator        * (const T num) const
    {
        return Matrix3<T>(a*num, b*num, c*num);
    }
    Matrix3<T> &operator        *= (const T num)
    {
        return *this = *this * num;
    }
    Matrix3<T> operator        / (const T num) const
    {
        return Matrix3<T>(a/num, b/num, c/num);
    }
    Matrix3<T> &operator        /= (const T num)
    {
        return *this = *this / num;
    }

    // 乘以一个向量
    Vector3<T> operator         *(const Vector3<T> &v) const;

    //矩阵的转置乘以一个向量
    Vector3<T>                  mul_transpose(const Vector3<T> &v) const;

    //提取x行向量
    Vector3<T>                  colx(void) const
    {
        return Vector3f(a.x, b.x, c.x);
    }

    //提取y行向量
    Vector3<T>        coly(void) const
    {
        return Vector3f(a.y, b.y, c.y);
    }

    //提取z行向量
    Vector3<T>        colz(void) const
    {
        return Vector3f(a.z, b.z, c.z);
    }
		
    //z行向量赋值
    void        set_colz( const Vector3<T> v)
    {
        a.z = v.x;	b.z = v.y; c.z = v.z;
    }		
		
    //和另一个3阶方阵相乘
    Matrix3<T> operator *(const Matrix3<T> &m) const;

    Matrix3<T> &operator        *=(const Matrix3<T> &m)
    {
        return *this = *this * m;
    }

    //矩阵转置
    Matrix3<T>          transposed(void) const;

    Matrix3<T>          transpose(void)
    {
        return *this = transposed();
    }

    //矩阵元素置零
    void        zero(void);

    //矩阵变为单位阵
    void        identity(void) {
        a.x = b.y = c.z = 1;
        a.y = a.z = 0;
        b.x = b.z = 0;
        c.x = c.y = 0;
    }

    //检查元素是否有异常值
    bool        is_nan(void)
    {
        return a.is_nan() || b.is_nan() || c.is_nan();
    }

    //欧拉角转余弦矩阵
    void        from_euler(const Vector3<T> &euler);
		
    //余弦矩阵转欧拉角
    void        to_euler(float *roll, float *pitch, float *yaw);

		//使用角速度向量来计算矩阵的一次旋转
    void        rotate(const Vector3<T> &g);

		//使用角速度向量(仅X，Y)来计算矩阵的一次旋转
    void        rotateXY(const Vector3<T> &g);
};

typedef Matrix3<int16_t>                Matrix3i;
typedef Matrix3<uint16_t>               Matrix3ui;
typedef Matrix3<int32_t>                Matrix3l;
typedef Matrix3<uint32_t>               Matrix3ul;
typedef Matrix3<float>                  Matrix3f;
typedef Matrix3<double>                 Matrix3d;

#endif // MATRIX3_H
