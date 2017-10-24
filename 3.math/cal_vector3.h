#ifndef CAL_VECTOR_H
#define CAL_VECTOR_H
#include <stdint.h>
#include <math.h>
template <typename T>
class Matrix3;
template <typename T>
class Vector3
{
	public :
		T x ,y ,z;
//构造函数，初始化向量值为0
    Vector3<T>() {
        x = y = z = 0;
    }
		
		//赋值构造函数
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }		
		
    void operator ()(const T x0, const T y0, const T z0)
    {
        x= x0; y= y0; z= z0;
    }		
		
		//向量旋转
		void rotate(enum Rotation rotation);
		
		//获取本向量平方
    T  length_squared() const
    {
        return (T)(*this * *this);
    }
		
		//获取本向量长度
    float length(void) const;
		
    //向量归一化
    void normalize()
    {
        *this /= length();
    }
		
    //向量置零
    void zero()
    {
        x = y = z = 0;
    }
		
    //返回归一化向量
    Vector3<T> normalized() const
    {
        return *this/length();
    }
		
	  //计算两个向量之间的夹角
    float angle(const Vector3<T> &v2) const;	
		
		void get_rollpitch(Vector3<T> &angle);
		
		void get_yaw(Vector3<T> &angle);	
		
    // 检查是否有元素的值为 NAN
    bool is_nan(void) const;		
		
/*************************运算符重载********************************/		
    //点积
    T operator *(const Vector3<T> &v) const;

    //叉积
    Vector3<T> operator %(const Vector3<T> &v) const;		
		
    //与标量相乘
    Vector3<T> operator *(const T num) const;
    Vector3<T> &operator *=(const T num);
		
    //与标量相除
    Vector3<T> operator  /(const T num) const;
    Vector3<T> &operator /=(const T num);
		
		//取反
	  Vector3<T> operator -(void) const;
	
	  //相加
    Vector3<T> operator +(const Vector3<T> &v) const;
    Vector3<T> &operator +=(const Vector3<T> &v);
		
    //相减
    Vector3<T> operator -(const Vector3<T> &v) const;
    Vector3<T> &operator -=(const Vector3<T> &v);
		
    //对比两个向量是否相等
    bool operator ==(const Vector3<T> &v) const;

    //对比两个向量是否不相等
    bool operator !=(const Vector3<T> &v) const;
		
    //矩阵乘以一个行向量，得到一个行向量
    Vector3<T> operator *(const Matrix3<T> &m) const;

    // 列向量乘以一个行向量,得到3x3矩阵
    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;
/*******************************************************************/													
		 
};


typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;



#endif